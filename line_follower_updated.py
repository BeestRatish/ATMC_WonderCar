#!/usr/bin/python3
# coding=utf8

import sys
import cv2
import time
import sys
sys.path.append('/home/pi/TurboPi/')
import math
import signal
import threading
import numpy as np
import yaml_handle
import HiwonderSDK.PID as PID
import HiwonderSDK.Misc as Misc
import HiwonderSDK.Board as Board
import HiwonderSDK.mecanum as mecanum
import HiwonderSDK.Sonar as Sonar
import Camera

# Initialize the Sonar module for ultrasonic sensing
HWSONAR = Sonar.Sonar()

# Global variables
servo1 = 1500
servo2 = 1500
img_centerx = 320
line_centerx = -1
size = (640, 480)
target_color = ()
__isRunning = False
swerve_pid = PID.PID(P=0.001, I=0.00001, D=0.000001)

car = mecanum.MecanumChassis()

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

lab_data = None
servo_data = None


# Function to load configuration data
def load_config():
    global lab_data, servo_data

    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)


# Initial position setup
def initMove():
    car_stop()
    Board.setPWMServoPulse(1, servo1, 1000)
    Board.setPWMServoPulse(2, servo2, 1000)


# Reset global variables
def reset():
    global line_centerx
    global target_color
    global servo1, servo2

    line_centerx = -1
    target_color = ()
    servo1 = servo_data['servo1'] + 350
    servo2 = servo_data['servo2']


# Initialization function
def init():
    print("VisualPatrol Init")
    load_config()
    reset()
    initMove()


# Start function
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("VisualPatrol Start")


# Stop function
def stop():
    global __isRunning
    __isRunning = False
    car_stop()
    print("VisualPatrol Stop")


# Exit function
def exit():
    global __isRunning
    __isRunning = False
    car_stop()
    print("VisualPatrol Exit")


# Set target color
def setTargetColor(color):
    global target_color

    print("COLOR", color)
    target_color = color
    return (True, ())


# Function to set the buzzer for a specified duration
def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)


# Stop all motors
def car_stop():
    car.set_velocity(0, 90, 0)


# Find the maximum contour in a list of contours
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= 5:
                area_max_contour = c

    return area_max_contour, contour_area_max


# Movement processing logic
car_en = False


def move():
    global line_centerx, car_en

    while True:
        if __isRunning:
            if line_centerx > 0:
                if abs(line_centerx - img_centerx) < 10:
                    line_centerx = img_centerx
                swerve_pid.SetPoint = img_centerx
                swerve_pid.update(line_centerx)
                angle = -swerve_pid.output
                car.set_velocity(50, 90, angle)
                car_en = True
        else:
            if car_en:
                car_en = False
                car_stop()
            time.sleep(0.01)


# Ultrasonic sensor obstacle detection and stopping function
def ultrasonic_check():
    global __isRunning

    while __isRunning:
        distance = HWSONAR.getDistance() / 10.0  # Distance in centimeters
        if distance <= 5:  # If obstacle is detected within 5 cm
            car_stop()  # Stop the robot
        time.sleep(0.1)  # Delay to avoid continuous checking and save resources


# Image processing and line following logic
def run(img):
    global line_centerx
    global target_color

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning or target_color == ():
        return img

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    centroid_x_sum = 0
    weight_sum = 0
    center_ = []
    n = 0

    # Image segmentation into upper, middle, and lower parts
    for r in roi:
        roi_h = roi_h_list[n]
        n += 1
        blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
        frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)
        area_max = 0
        areaMaxContour = 0
        for i in target_color:
            if i in lab_data:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[i]['min'][0],
                                          lab_data[i]['min'][1],
                                          lab_data[i]['min'][2]),
                                         (lab_data[i]['max'][0],
                                          lab_data[i]['max'][1],
                                          lab_data[i]['max'][2]))
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

        cnts = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]
        cnt_large, area = getAreaMaxContour(cnts)
        if cnt_large is not None:
            rect = cv2.minAreaRect(cnt_large)
            box = np.int0(cv2.boxPoints(rect))
            for i in range(4):
                box[i, 1] = box[i, 1] + (n - 1) * roi_h + roi[0][0]
                box[i, 1] = int(Misc.map(box[i, 1], 0, size[1], 0, img_h))
            for i in range(4):
                box[i, 0] = int(Misc.map(box[i, 0], 0, size[0], 0, img_w))

            cv2.drawContours(img, [box], -1, (0, 0, 255, 255), 2)

            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2
            cv2.circle(img, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
            center_.append([center_x, center_y])
            centroid_x_sum += center_x * r[4]
            weight_sum += r[4]
        if weight_sum != 0:
            cv2.circle(img, (line_centerx, int(center_y)), 10, (0, 255, 255), -1)
            line_centerx = int(centroid_x_sum / weight_sum)
        else:
            line_centerx = -1
        return img


def manual_stop(signum, frame):
    global __isRunning

    print('Closing...')
    __isRunning = False
    car_stop()


if __name__ == '__main__':
    init()
    start()
    __isRunning = True
    target_color = ('black',)
    camera = Camera.Camera()
    camera.camera_open(correction=True)
    signal.signal(signal.SIGINT, manual_stop)

    # Ultrasonic sensor thread
    ultrasonic_thread = threading.Thread(target=ultrasonic_check)
    ultrasonic_thread.setDaemon(True)
    ultrasonic_thread.start()

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
