#!/usr/bin/python3
# coding=utf8

import sys
import cv2
import time
import math
import signal
import threading
import numpy as np
import yaml_handle
import HiwonderSDK.PID as PID
import HiwonderSDK.Misc as Misc
import HiwonderSDK.Board as Board
import HiwonderSDK.mecanum as mecanum
import Camera

# Add TurboPi library to the Python path
sys.path.append('/home/pi/TurboPi/')

# Constants and global variables
servo1 = 1500  # Initial servo positions
servo2 = 1500
img_centerx = 320  # Image center
line_centerx = -1  # Default line center
size = (640, 480)  # Image size
target_color = ()  # Target color for line following
__isRunning = False  # Flag to control program execution
swerve_pid = PID.PID(P=0.001, I=0.00001, D=0.000001)  # PID controller for swerve

car = mecanum.MecanumChassis()  # Initialize the mecanum chassis

# Define RGB values for different colors
range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# ROI (Region of Interest) settings for image processing
roi = [
    (240, 280, 0, 640, 0.1),
    (340, 380, 0, 640, 0.3),
    (430, 460, 0, 640, 0.6)
]

roi_h1 = roi[0][0]
roi_h2 = roi[1][0] - roi[0][0]
roi_h3 = roi[2][0] - roi[1][0]
roi_h_list = [roi_h1, roi_h2, roi_h3]

# Load configuration data from YAML files
lab_data = None
servo_data = None

def load_config():
    global lab_data, servo_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

# Initialize the hardware and configuration
def init():
    print("VisualPatrol Init")
    load_config()
    initMove()

# Move the robot to its initial position
def initMove():
    car_stop()  # Stop the car
    Board.setPWMServoPulse(1, servo1, 1000)  # Set servo positions
    Board.setPWMServoPulse(2, servo2, 1000)

# Reset variables and settings
def reset():
    global line_centerx, target_color, servo1, servo2
    line_centerx = -1
    target_color = ()
    servo1 = servo_data['servo1'] + 350
    servo2 = servo_data['servo2']

# Start line following behavior
def start():
    global __isRunning
    reset()  # Reset settings
    __isRunning = True
    print("VisualPatrol Start")

# Stop line following behavior
def stop():
    global __isRunning
    __isRunning = False
    car_stop()  # Stop the car
    print("VisualPatrol Stop")

# Exit the program
def exit():
    global __isRunning
    __isRunning = False
    car_stop()  # Stop the car
    print("VisualPatrol Exit")

# Set the target color for line following
def setTargetColor(color):
    global target_color
    print("COLOR", color)
    target_color = color
    return (True, ())

# Set the buzzer for a specific duration
def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)

# Stop all motors (car movement)
def car_stop():
    car.set_velocity(0, 90, 0)  # Stop all motors

# Find the maximum contour within a list of contours
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate contour area
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= 5:  # Filter out small contours
                area_max_contour = c

    return area_max_contour, contour_area_max

# Main movement logic for the robot
car_en = False
def move():
    global line_centerx, car_en

    while True:
        if __isRunning:
            if line_centerx > 0:
                # Check if the deviation is small enough, otherwise use PID control
                if abs(line_centerx - img_centerx) < 10:
                    line_centerx = img_centerx
                swerve_pid.SetPoint = img_centerx  # Set PID setpoint
                swerve_pid.update(line_centerx)  # Update PID controller
                angle = -swerve_pid.output  # Get PID output
                car.set_velocity(50, 90, angle)  # Set car velocity
                car_en = True  # Flag to indicate car movement
        else:
            if car_en:
                car_en = False
                car_stop()  # Stop the car
            time.sleep(0.01)

# Start the movement thread
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

# Image processing and line following logic
def run(img):
    global line_centerx, target_color

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning or target_color == ():
        return img

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)  # Resize the image
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)  # Apply Gaussian blur
    centroid_x_sum = 0
    weight_sum = 0
    center_ = []
    n = 0

    # Process each ROI separately
    for r in roi:
        roi_h = roi_h_list[n]
        n += 1
        blobs = frame_gb[r[0]:r[1], r[2]:r[3]]  # Extract ROI
        frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)  # Convert to LAB color space

        for i in target_color:
            if i in lab_data:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab, lab_data[i]['min'], lab_data[i]['max'])  # Create color mask
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # Erosion
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # Dilation

        cnts = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # Find contours
        cnt_large, area = getAreaMaxContour(cnts)  # Get the largest contour
        if cnt_large is not None:
            rect = cv2.minAreaRect(cnt_large)  # Minimum bounding rectangle
            box = np.int0(cv2.boxPoints(rect))  # Get rectangle corners

            # Adjust coordinates to the original image size
            for i in range(4):
                box[i, 1] = box[i, 1] + (n - 1) * roi_h + roi[0][0]
                box[i, 1] = int(Misc.map(box[i, 1], 0, size[1], 0, img_h))
            for i in range(4):
                box[i, 0] = int(Misc.map(box[i, 0], 0, size[0], 0, img_w))

            cv2.drawContours(img, [box], -1, (0, 0, 255, 255), 2)  # Draw the rectangle
            pt1_x, pt1_y = box[0, 0], box[0, 1]  # Top-left corner
            pt3_x, pt3_y = box[2, 0], box[2, 1]  # Bottom-right corner
            center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2  # Calculate center
            cv2.circle(img, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)  # Draw the center point
            center_.append([center_x, center_y])  # Store the center coordinates
            centroid_x_sum += center_x * r[4]  # Weighted sum of x-coordinates
            weight_sum += r[4]  # Total weight of centers

    if weight_sum != 0:
        line_centerx = int(centroid_x_sum / weight_sum)  # Calculate the final center point
    else:
        line_centerx = -1

    return img

# Signal handler for manual program stop
def manual_stop(signum, frame):
    global __isRunning
    print('Shutting down...')
    __isRunning = False
    car_stop()  # Stop the car

if __name__ == '__main__':
    init()  # Initialize the program
    start()  # Start line following
    __isRunning = True
    target_color = ('black',)  # Set target color
    camera = Camera.Camera()  # Initialize the camera
    camera.camera_open(correction=True)  # Open the camera with distortion correction
    signal.signal(signal.SIGINT, manual_stop)  # Register signal handler for manual stop

    while __isRunning:
        img = camera.frame  # Get the camera frame
        if img is not None:
            frame = img.copy()
            frame = run(frame)  # Process the frame
            frame_resize = cv2.resize(frame, (320, 240))  # Resize for display
            cv2.imshow('frame', frame_resize)  # Show the processed frame
            key = cv2.waitKey(1)
            if key == 27:  # Check for ESC key press
                break
        else:
            time.sleep(0.01)

    camera.camera_close()  # Close the camera
    cv2.destroyAllWindows()  # Close OpenCV windows
