#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import time
import math
import signal
import numpy as np
import yaml_handle
import HiwonderSDK.Board as Board
import HiwonderSDK.mecanum as mecanum
import HiwonderSDK.FourInfrared as infrared
import Camera
import threading

# Define speed limit
speed_limit_cascade = cv2.CascadeClassifier('speedlimit.xml')

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


# Initial Position
def initMove():
    car.set_velocity(0, 90, 0)
    Board.setPWMServoPulse(1, servo1, 1000)
    Board.setPWMServoPulse(2, servo2, 1000)


# Set Buzzer
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


# Reset Variables
def reset():
    global car_stop, color_list, detect_color, servo1, servo2

    car_stop = False
    color_list = []
    detect_color = 'None'
    servo1 = servo_data['servo1']
    servo2 = servo_data['servo2']


# APP Initialization
def init():
    print("LineFollower Init")
    load_config()
    reset()
    initMove()


# App starts calling game program
def start():
    global __isRunning
    reset()
    __isRunning = True
    car.set_velocity(20, 90, 0)
    print("LineFollower Start")


# App stops calling game program
def stop():
    global car_stop, __isRunning
    car_stop = True
    __isRunning = False
    set_rgb('None')
    print("LineFollower Stop")


# Exit the game
def exit():
    global car_stop, __isRunning
    car_stop = True
    __isRunning = False
    set_rgb('None')
    print("LineFollower Exit")


def setTargetColor(color):
    global target_color

    target_color = color
    return (True, ())


# Set RGB color on the expansion board consistent with the tracked color
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


# Find the maximum contour
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:
                area_max_contour = c

    return area_max_contour, contour_area_max


def move():
    global car_stop, __isRunning, detect_color

    while True:
        if __isRunning:
            if detect_color != 'red':
                set_rgb(detect_color)
                sensor_data = line.readData()
                if not sensor_data[0] and sensor_data[1] and sensor_data[2] and not sensor_data[3]:
                    car.set_velocity(35, 90, 0)
                    car_stop = True
                elif not sensor_data[0] and not sensor_data[1] and sensor_data[2] and not sensor_data[3]:
                    car.set_velocity(35, 90, 0.03)
                    car_stop = True
                elif not sensor_data[0] and sensor_data[1] and not sensor_data[2] and not sensor_data[3]:
                    car.set_velocity(35, 90, -0.03)
                    car_stop = True
                elif not sensor_data[0] and not sensor_data[1] and not sensor_data[2] and sensor_data[3]:
                    car.set_velocity(35, 90, 0.3)
                    car_stop = True
                elif sensor_data[0] and not sensor_data[1] and not sensor_data[2] and not sensor_data[3]:
                    car.set_velocity(35, 90, -0.3)
                    car_stop = True

                elif sensor_data[0] and sensor_data[1] and sensor_data[2] and sensor_data[3]:
                    if car_stop:
                        car.set_velocity(0, 90, 0)
                        car_stop = False
                    time.sleep(0.01)

                if detect_color == 'green':
                    if not car_stop:
                        car.set_velocity(35, 90, 0)
                        car_stop = True

            else:
                if car_stop:
                    setBuzzer(0.1)
                    set_rgb(detect_color)
                    car.set_velocity(0, 90, 0)
                    car_stop = False
                time.sleep(0.01)

        else:
            if car_stop:
                car.set_velocity(0, 90, 0)
                car_stop = False
            time.sleep(0.01)


# Run child thread
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()


# Image processing
def run(img):
    global __isRunning, detect_color, color_list

    if not __isRunning:
        return img

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

    # Speed limit detection
    gray = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2GRAY)
    speed_limit_signs = speed_limit_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    if len(speed_limit_signs) > 0:
        car.set_velocity(50, 90, 0)
    else:
        max_area = 0
        color_area_max = None
        areaMaxContour_max = 0

        for i in target_color:
            if i in lab_data:
                frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[i]['min'][0], lab_data[i]['min'][1], lab_data[i]['min'][2]),
                                         (lab_data[i]['max'][0], lab_data[i]['max'][1], lab_data[i]['max'][2]))

                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = getAreaMaxContour(contours)

                if areaMaxContour is not None:
                    if area_max > max_area:
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour

        if max_area > 2500:
            rect = cv2.minAreaRect(areaMaxContour_max)
            box = np.int0(cv2.boxPoints(rect))
            cv2.drawContours(img, [box], -1, range_rgb[color_area_max], 2)

            if color_area_max == 'red':
                color = 1
            elif color_area_max == 'green':
                color = 2
            else:
                color = 0
            color_list.append(color)

            if len(color_list) == 3:
                color = np.mean(np.array(color_list))
                color_list = []
                if color == 1:
                    detect_color = 'red'
                elif color == 2:
                    detect_color = 'green'
                else:
                    detect_color = 'None'

    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)

    return img


def manualcar_stop(signum, frame):
    global __isRunning

    print('Shutting down...')
    __isRunning = False
    car.set_velocity(0, 90, 0)


if __name__ == '__main__':
    init()
    start()
    camera = Camera.Camera()
    camera.camera_open(correction=True)

    signal.signal(signal.SIGINT, manualcar_stop)

    while __isRunning:
        img = camera.frame
        if img is not None:
            frame = img.copy()
            frame_processed = run(frame)
            frame_resized = cv2.resize(frame_processed, (320, 240))
            cv2.imshow('frame', frame_resized)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)

    camera.camera_close()
    cv2.destroyAllWindows()

