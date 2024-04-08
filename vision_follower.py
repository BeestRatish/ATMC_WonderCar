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
import HiwonderSDK.FourInfrared as infrared  # Import the line detection module

# Initialize line detector
line = infrared.FourInfrared()

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


def load_config():
    global lab_data, servo_data

    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)


def initMove():
    car_stop()
    Board.setPWMServoPulse(1, servo1, 1000)
    Board.setPWMServoPulse(2, servo2, 1000)


def reset():
    global line_centerx
    global target_color
    global servo1, servo2

    line_centerx = -1
    target_color = ()
    servo1 = servo_data['servo1'] + 350
    servo2 = servo_data['servo2']


def init():
    print("VisualPatrol Init")
    load_config()
    reset()
    initMove()


def start():
    global __isRunning
    reset()
    __isRunning = True
    print("VisualPatrol Start")


def stop():
    global __isRunning
    __isRunning = False
    car_stop()
    print("VisualPatrol Stop")


def exit():
    global __isRunning
    __isRunning = False
    car_stop()
    print("VisualPatrol Exit")


def setTargetColor(color):
    global target_color

    print("COLOR", color)
    target_color = color
    return (True, ())


def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)


def car_stop():
    car.set_velocity(0, 90, 0)  # Stop all motors


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
                car_stop()
                car_en = False
        else:
            if car_en:
                car_stop()
                car_en = False
            time.sleep(0.01)


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


def run(img):
    global line_centerx

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

    for r in roi:
        h1, h2, w1, w2, k = r
        frame_roi = frame_gb[h1:h2, w1:w2]
        frame_lab = cv2.cvtColor(frame_roi, cv2.COLOR_BGR2LAB)
        frame_mask = cv2.inRange(frame_lab, lab_data[target_color]['min'], lab_data[target_color]['max'])
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        areaMaxContour, area_max = getAreaMaxContour(contours)

        if areaMaxContour is not None:
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))

            cv2.drawContours(img, [box], -1, range_rgb[target_color], 2)

            M = cv2.moments(areaMaxContour)
            c_x, c_y = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

            cv2.circle(img, (c_x, c_y), 3, (0, 255, 0), 2)
            cv2.putText(img, "centroid", (c_x - 25, c_y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            centroid_x_sum += c_x * k
            weight_sum += k

            center_.append((c_x, c_y))

            n += 1

    if n == 0:
        return img

    line_centerx = int(centroid_x_sum / weight_sum)

    cv2.circle(img, (line_centerx, img_centerx), 5, (255, 0, 0), -1)
    cv2.putText(img, "LineCenterX: " + str(line_centerx), (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                (0, 255, 255), 2)

    return img


def manualcar_stop(signum, frame):
    global __isRunning

    print('Closing...')
    __isRunning = False
    car_stop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--color", type=str, help="Target color (red, green, blue)")
    args = parser.parse_args()

    if args.color:
        color = args.color.lower()
        if color in range_rgb:
            init()
            setTargetColor(color)
            start()

            camera = Camera.Camera()
            camera.camera_open(correction=True)
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
        else:
            print("Invalid color argument! Use 'red', 'green', or 'blue'.")
    else:
        print("Please provide a color argument! Use --color <color> (e.g., --color red).")
