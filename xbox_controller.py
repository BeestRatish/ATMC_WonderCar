#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/TurboPi/')
import time
import signal
import pygame
import HiwonderSDK.mecanum as mecanum

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

pygame.init()
controller = pygame.joystick.Joystick(0)
controller.init()

print('''
**********************************************************
****************功能:小车前后左右移动及转向例程 Function: Car Movement and Turning Control*********************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！ Press Ctrl+C to exit the program, please try few more times if fail to exit!
----------------------------------------------------------
''')

chassis = mecanum.MecanumChassis()

start = True
#关闭前处理 Processing before exit
def Stop(signum, frame):
    global start

    start = False
    print('Closing...')
    chassis.set_velocity(0,0,0)  # 关闭所有电机 Turn off all motors

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    while start:
        pygame.event.get()
        linear_speed = int(controller.get_axis(1) * 100)  # Left joystick Y-axis for linear speed
        direction_angle = int((controller.get_axis(0) + 1) * 180)  # Left joystick X-axis for direction angle
        yaw_speed = controller.get_axis(2) * 2  # Right joystick X-axis for yaw speed

        chassis.set_velocity(linear_speed, direction_angle, yaw_speed)
        time.sleep(0.1)  # Adjust sleep time as needed
    chassis.set_velocity(0, 0, 0)  # Turn off all motors
    print('Closed')
