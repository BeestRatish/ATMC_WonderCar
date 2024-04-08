import sys
sys.path.append('/home/pi/TurboPi/')
import cv2
import time
import signal
import HiwonderSDK.mecanum as mecanum

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

chassis = mecanum.MecanumChassis()

start = True

# Signal handler for stopping the program
def Stop(signum, frame):
    global start
    start = False
    print('Closing...')
    chassis.set_velocity(0, 0, 0)  # Turn off all motors

signal.signal(signal.SIGINT, Stop)

# Placeholder function for AI vision model (replace with actual implementation)
def ai_vision(frame):
    # Placeholder for distance estimation
    return 0.15  # Simulated distance of 15 cm

if __name__ == '__main__':
    while start:
        chassis.set_velocity(50, 90, 0)  # Move forward at linear velocity 50, direction angle 90
        time.sleep(1)

        # Capture a frame from the camera
        _, frame = cap.read()

        # Convert the frame to grayscale for processing
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect stop signs in the frame
        stop_signs = stop_sign_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        for (x, y, w, h) in stop_signs:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            # Calculate distance to stop sign and check if it's within 15 cm
            distance = ai_vision(frame)
            if distance <= 0.15:
                # Stop the car for 3 seconds
                chassis.set_velocity(0, 0, 0)
                time.sleep(3)
                chassis.set_velocity(50, 90, 0)  # Resume movement after 3 seconds

        cv2.imshow('Car Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    chassis.set_velocity(0, 0, 0)  # Turn off all motors
    print('Closed')
