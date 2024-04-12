#!/usr/bin/python3
# coding=utf8
import sys

sys.path.append('/home/pi/TurboPi/')
import time
import signal
import cv2
import HiwonderSDK.mecanum as mecanum

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

print('''
**********************************************************
********************功能:小车前进例程 Function: Move Forward************************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！ Press Ctrl+C to exit the program, please try few more times if fail to exit!
----------------------------------------------------------
''')

# Import the stop sign detection code
# Define the file paths for the Haar Cascade classifiers
stop_cascade_path = 'stopsign_classifier.xml'
# Define the real-world dimensions of the stop sign (in meters)
stop_sign_width = 0.45  # Width of the stop sign in meters (e.g., 45 cm)
stop_sign_height = 0.45  # Height of the stop sign in meters (e.g., 45 cm)
# Define the focal length of the camera (in pixels) and the actual distance from the camera to the stop sign (in meters)
focal_length = 1000  # Focal length of the camera in pixels (adjust based on your camera)
actual_distance = 1.5  # Actual distance from the camera to the stop sign in meters

# Load the pre-trained Haar Cascade classifier for stop signs
stop_cascade = cv2.CascadeClassifier(stop_cascade_path)


# Function to detect stop signs
def detect_stop_sign(frame):
    # Convert the frame to grayscale for detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect stop signs in the frame
    stop_signs = stop_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # Check if any stop signs are detected within the specified distance
    for (x, y, w, h) in stop_signs:
        # Calculate the distance based on the size of the stop sign in the image
        stop_sign_width_px = w  # Width of the stop sign in pixels
        estimated_distance = (stop_sign_width * focal_length) / stop_sign_width_px

        # If a stop sign is within the specified distance, return True
        if estimated_distance <= actual_distance:
            return True

    # If no stop sign is within the specified distance, return False
    return False


chassis = mecanum.MecanumChassis()

start = True


# 关闭前处理 Processing before exit
def Stop(signum, frame):
    global start

    start = False
    print('closing...')
    chassis.set_velocity(0, 0, 0)  # 关闭所有电机 Turn off all motors


signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    while start:
        ret, frame = cap.read()
        if not ret:
            break

        # Check for stop signs in the frame
        if detect_stop_sign(frame):
            chassis.set_velocity(0, 0, 0)  # Stop the robot
            print('Stop sign detected. Stopping the robot.')
            time.sleep(1)  # Wait for 1 second

        else:
            chassis.set_velocity(50, 90, 0)  # Move the robot forward
            time.sleep(1)

    chassis.set_velocity(0, 0, 0)  # 关闭所有电机 Turn off all motors
    print('Closed')

    # Release the camera
    cap.release()
    cv2.destroyAllWindows()
