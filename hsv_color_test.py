import cv2
import numpy as np

# Try to open the default camera using DirectShow on Windows, or V4L2 on Linux
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Use CAP_V4L2 for Linux if necessary

# Check if camera is opened successfully
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Set resolution for the camera to avoid memory allocation issues
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Press 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Convert the frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Draw a rectangle at the center to sample the color
    height, width, _ = frame.shape
    cx, cy = width // 2, height // 2
    size = 10
    cv2.rectangle(frame, (cx - size, cy - size), (cx + size, cy + size), (255, 255, 255), 2)

    # Get the HSV value of the pixel at the center
    hsv_value = hsv[cy, cx]
    text = f"H: {hsv_value[0]} S: {hsv_value[1]} V: {hsv_value[2]}"
    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow('HSV Color Tester', frame)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
