import sys

sys.path.append('/home/pi/TurboPi/')
import cv2
import time
import signal
import HiwonderSDK.mecanum as mecanum

# Define the file path for the Haar Cascade classifier
cascade_path = 'stopsign_classifier.xml'  # Replace 'stopsign_classifier.xml' with the actual file path

# Define the real-world dimensions of the stop sign (in meters)
stop_sign_width = 0.45  # Width of the stop sign in meters (e.g., 45 cm)
stop_sign_height = 0.45  # Height of the stop sign in meters (e.g., 45 cm)
# Define the focal length of the camera (in pixels) and the actual distance from the camera to the stop sign (in meters)
focal_length = 1000  # Focal length of the camera in pixels (adjust based on your camera)

# Initialize the camera with the specified resolution
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# Load the pre-trained Haar Cascade classifier for stop signs
stop_cascade = cv2.CascadeClassifier(cascade_path)

# Initialize the Mecanum chassis
chassis = mecanum.MecanumChassis()

start = True

# Signal handler for stopping the program
def Stop(signum, frame):
    global start
    start = False
    print('Closing...')
    chassis.set_velocity(0, 0, 0)  # Turn off all motors

signal.signal(signal.SIGINT, Stop)

# Function for object detection and distance estimation
def process_frame(frame):
    # Convert the frame to grayscale for detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect stop signs in the frame
    stop_signs = stop_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    for (x, y, w, h) in stop_signs:
        # Draw rectangles around the detected stop signs
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # Calculate the distance based on the size of the stop sign in the image
        stop_sign_width_px = w  # Width of the stop sign in pixels

        # Estimate distance using the pinhole camera model (assuming focal length is known)
        estimated_distance = (stop_sign_width * focal_length) / stop_sign_width_px

        # Display the estimated distance on the frame
        cv2.putText(frame, f'Distance: {estimated_distance:.2f} meters', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the frame with detected stop signs and distance estimation
    cv2.imshow('Stop Signs Detected', frame)

if __name__ == '__main__':
    while start:
        chassis.set_velocity(50, 90, 0)  # Move forward with specified parameters
        time.sleep(1)

        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            break

        # Process frames using the process_frame function
        process_frame(frame)

        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Turn off motors and close the program
    chassis.set_velocity(0, 0, 0)
    cap.release()
    cv2.destroyAllWindows()
    print('Closed')
