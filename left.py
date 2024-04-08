import cv2

# Define the file path for the Haar Cascade classifier
cascade_path = 'left.xml'  # Replace 'stopsign_classifier.xml' with the actual file path
# Define the real-world dimensions of the stop sign (in meters)
left_sign_width = 0.45  # Width of the stop sign in meters (e.g., 45 cm)
left_sign_height = 0.45  # Height of the stop sign in meters (e.g., 45 cm)
# Define the focal length of the camera (in pixels) and the actual distance from the camera to the stop sign (in meters)
focal_length = 1000  # Focal length of the camera in pixels (adjust based on your camera)
actual_distance = 1.5  # Actual distance from the camera to the stop sign in meters

# Set the camera resolution
resolution = (640, 480)

# Initialize the camera with the specified resolution
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

# Load the pre-trained Haar Cascade classifier for stop signs
left_cascade = cv2.CascadeClassifier(cascade_path)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame to grayscale for detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect stop signs in the frame
    left_signs = left_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # Draw rectangles around the detected stop signs and estimate distance
    for (x, y, w, h) in left_signs:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # Calculate the distance based on the size of the stop sign in the image
        left_sign_width_px = w  # Width of the stop sign in pixels
        left_sign_height_px = h  # Height of the stop sign in pixels

        # Estimate distance using the pinhole camera model (assuming focal length is known)
        estimated_distance = (left_sign_width * focal_length) / left_sign_width_px

        # Display the estimated distance on the frame
        cv2.putText(frame, f'Distance: {estimated_distance:.2f} meters', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the frame with detected stop signs and distance estimation
    cv2.imshow('Left Signs Detected', frame)

    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()

