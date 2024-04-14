#!/usr/bin/python3
# coding=utf8
import sys

sys.path.append('/home/pi/TurboPi/')
import cv2
import time
import math
import signal
import Camera
import argparse
import threading
import numpy as np
import yaml_handle
import HiwonderSDK.PID as PID
import HiwonderSDK.Misc as Misc
import HiwonderSDK.Board as Board
import HiwonderSDK.mecanum as mecanum

# Load the stop sign HAAR cascade classifier
stop_cascade = cv2.CascadeClassifier('stopsign_classifier.xml')
speedlimit_cascade = cv2.CascadeClassifier('speedlimit.xml')

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


# Load configuration from YAML files
def load_config():
    global lab_data, servo_data

    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)


# Initialize movement
def initMove():
    car_stop()
    Board.setPWMServoPulse(1, servo1, 1000)
    Board.setPWMServoPulse(2, servo2, 1000)

def adjust_velocity(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    speedlimit_signs = speedlimit_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    if len(speedlimit_signs) > 0:
        car.set_velocity(50,90,0)
    else:
        car_run()

# Reset variables
def reset():
    global line_centerx
    global target_color
    global servo1, servo2

    line_centerx = -1
    target_color = ()
    servo1 = servo_data['servo1'] + 350
    servo2 = servo_data['servo2']


# App initialization
def init():
    print("VisualPatrol Init")
    load_config()
    reset()
    initMove()


# Start visual patrol
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("VisualPatrol Start")


# Stop visual patrol
def stop():
    global __isRunning
    __isRunning = False
    car_stop()
    print("VisualPatrol Stop")


# Exit visual patrol
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


# Set buzzer
def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)


# Stop the car's motors
def car_stop():
    car.set_velocity(0, 90, 0)


# Find the maximum contour
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


# Movement logic
car_en = False


def move():
    global line_centerx, car_en, stop_detected

    while True:
        if __isRunning:
            if line_centerx > 0:
                if abs(line_centerx - img_centerx) < 10:
                    line_centerx = img_centerx
                swerve_pid.SetPoint = img_centerx
                swerve_pid.update(line_centerx)
                angle = -swerve_pid.output
                car.set_velocity(30, 90, angle)
                car_en = True


                #check if a stop sign is detected
                if stop_detected:
                    car_stop()
                    while stop_detected:
                        time.sleep(0.1)
                    car_run()
        else:
            if car_en:
                car_en = False
                car_stop()
            time.sleep(0.01)


# Run child thread for movement
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

roi = [
    (240, 280, 0, 640, 0.1),
    (340, 380, 0, 640, 0.3),
    (430, 460, 0, 640, 0.6)
]

roi_h1 = roi[0][0]
roi_h2 = roi[1][0] - roi[0][0]
roi_h3 = roi[2][0] - roi[1][0]

roi_h_list = [roi_h1, roi_h2, roi_h3]

#image processing
def run(img):
    global line_centerx
    global target_color
    global __isRunning

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

    # Lane detection (black line following)
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

    # Stop sign detection
    gray = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2GRAY)
    stop_signs = stop_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    if len(stop_signs) > 0:
        # Stop the car when a stop sign is detected
        car_stop()
        time.sleep(1)
    else:
        # Resume running if no stop sign is detected
        car_run()

    adjust_velocity(img)

    return img


# Function to stop the car
def car_stop():
    global __isRunning
    __isRunning = False
    car.set_velocity(0, 90, 0)  # Stop all motors


# Function to resume running the car
def car_run():
    global __isRunning
    __isRunning = True
    # Add code here to set appropriate velocity for car movement
    # For example:
    # car.set_velocity(50, 90, angle)


# Stop the car and print closing message
def manual_stop(signum, frame):
    global __isRunning

    print('Shutting down...')
    __isRunning = False
    car_stop()


# Main program
if __name__ == '__main__':
    init()
    start()
    __isRunning = True
    target_color = ('black',)
    camera = Camera.Camera()
    camera.camera_open(correction=True)  # Enable distortion correction
    signal.signal(signal.SIGINT, manual_stop)
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
