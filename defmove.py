#!/usr/bin/python3
# coding=utf8
import sys

sys.path.append('/home/pi/TurboPi/')
import cv2
import time
import math
import signal
import Camera
import threading
import numpy as np
import yaml_handle
import HiwonderSDK.Board as Board
import HiwonderSDK.mecanum as mecanum
import HiwonderSDK.FourInfrared as infrared

# Load other modules and define global variables as before...

# Load the stop sign classifier
stop_sign_cascade = cv2.CascadeClassifier('stop_sign_cascade.xml')

# Add a global variable to track if the car should move or stop
should_move = True

# 红绿灯行驶

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

car = mecanum.MecanumChassis()
line = infrared.FourInfrared()

servo1 = 1500
servo2 = 1500
car_stop = False
color_list = []
size = (640, 480)
__isRunning = False
detect_color = 'None'
target_color = ('red', 'green')

lab_data = None
servo_data = None


def load_config():
    global lab_data, servo_data

    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)


# 初始位置
def initMove():
    car.set_velocity(0, 90, 0)
    Board.setPWMServoPulse(1, servo1, 1000)
    Board.setPWMServoPulse(2, servo2, 1000)


# 设置蜂鸣器
def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)


range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

draw_color = range_rgb["black"]


# 变量重置
def reset():
    global car_stop
    global color_list
    global detect_color
    global should_move

    car_stop = False
    color_list = []
    detect_color = 'None'
    should_move = True


# app初始化调用
def init():
    print("LineFollower Init")
    load_config()
    reset()
    initMove()


# app开始玩法调用
def start():
    global __isRunning
    reset()
    __isRunning = True
    car.set_velocity(25, 90, 0)
    print("LineFollower Start")


# app停止玩法调用
def stop():
    global car_stop
    global __isRunning
    car_stop = True
    __isRunning = False
    set_rgb('None')
    print("LineFollower Stop")


# app退出玩法调用
def exit():
    global car_stop
    global __isRunning
    car_stop = True
    __isRunning = False
    set_rgb('None')
    print("LineFollower Exit")


def setTargetColor(color):
    global target_color

    target_color = color
    return (True, ())


# 设置扩展板的RGB灯颜色使其跟要追踪的颜色一致
def set_rgb(color):
    if color == "red":
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
        Board.RGB.show()
    elif color == "green":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
        Board.RGB.show()
    elif color == "blue":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
        Board.RGB.show()
    else:
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()


# 找出面积最大的轮廓
# 参数为要比较的轮廓的列表
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓


def move():
    global car_stop
    global __isRunning
    global detect_color
    global should_move

    while True:
        if __isRunning:
            if should_move:  # Move the car if should_move is True
                if detect_color != 'red':
                    set_rgb(detect_color)  # Set the RGB color based on the detected color
                    sensor_data = line.readData()  # Read data from 4 infrared sensors
                    # Add your existing car movement logic here
                    pass
            else:  # Stop the car if should_move is False
                car.set_velocity(0, 90, 0)  # Stop the car
                time.sleep(0.01)  # Add a small delay to reduce CPU usage

        else:
            if car_stop:
                car.set_velocity(0, 90, 0)  # Stop the car
                car_stop = False
            time.sleep(0.01)


# 运行子线程
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()


# 机器人图像处理
def run(img):
    global __isRunning
    global detect_color, draw_color, color_list
    global should_move

    if not __isRunning:  # Check if the game is running, return the original image if not
        return img

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    # Perform color and stop sign detection here
    # You can add more conditions based on your needs
    # For simplicity, this code assumes stop signs are red rectangles
    gray = cv2.cvtColor(img_copy, cv2.COLOR_BGR2GRAY)
    stop_signs = stop_sign_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)

    if len(stop_signs) > 0:  # Stop sign detected
        should_move = False
        for (x, y, w, h) in stop_signs:
            cv2.rectangle(img_copy, (x, y), (x + w, y + h), (0, 0, 255), 2)
            detect_color = 'red'  # Assuming stop signs are red
            draw_color = (0, 0, 255)  # Red color for drawing
            car.set_velocity(0, 90, 0)  # Stop the car when a stop sign is detected
            time.sleep(3)  # Wait for 3 seconds before resuming
    else:  # No stop sign detected
        should_move = True
        detect_color = 'None'
        draw_color = range_rgb["black"]

    cv2.putText(img_copy, "Color: " + detect_color, (10, img_h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)

    return img_copy


# 关闭前处理
def manualcar_stop(signum, frame):
    global __isRunning

    print('关闭中...')
    __isRunning = False
    car.set_velocity(0, 90, 0)  # 关闭所有电机


if __name__ == '__main__':
    init()
    start()
    camera = Camera.Camera()
    camera.camera_open(correction=True)  # 开启畸变矫正，默认不开启
    signal.signal(signal.SIGINT, manualcar_stop)
    while __isRunning:
        img = camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)
            frame_resize = cv2.resize(Frame, (320, 240))
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    camera.camera_close()
    cv2.destroyAllWindows()
