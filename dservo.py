#durv servoline.py edit
from picamera2 import Picamera2
import cv2
import numpy as np
import RPi.GPIO as GPIO
import os
import time

# Initialize camera
picam2 = Picamera2()
config = picam2.create_preview_configuration()
picam2.configure(config)
picam2.start()

# Define motor driver GPIO pins
motor_in1, motor_in2 = 26, 6  # Left motor
motor_in3, motor_in4 = 16, 5  # Right motor
ENA, ENB = 19, 13  # PWM speed control

# Define servo GPIO pin
servo_pin = 18  # Servo motor connected to GPIO 18

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup([motor_in1, motor_in2, motor_in3, motor_in4, ENA, ENB, servo_pin], GPIO.OUT)

# Setup PWM for motors
pwm1, pwm2 = GPIO.PWM(ENA, 1000), GPIO.PWM(ENB, 1000)
pwm1.start(0)
pwm2.start(0)

# Setup PWM for servo (50 Hz for standard servo motor)
servo_pwm = GPIO.PWM(servo_pin, 50)
servo_pwm.start(0)  
time.sleep(0.5)  # Allow the servo to reset

# ✓ Use your calibrated servo values
CENTER_DUTY = 5.0  # Neutral position # 5 may not be exactly centre (∴5.5)
LEFT_DUTY = 9.0  # Leftmost position
RIGHT_DUTY = 1.0   # Rightmost position

# Move servo to center and stop signal
servo_pwm.ChangeDutyCycle(CENTER_DUTY)  
time.sleep(1)
servo_pwm.ChangeDutyCycle(0)

# Speed settings
move_speed = 55
turn_speed = 85

def set_speed(speed):
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)

def move_left():
    GPIO.output(motor_in1, GPIO.LOW)
    GPIO.output(motor_in2, GPIO.HIGH)
    GPIO.output(motor_in3, GPIO.HIGH)
    GPIO.output(motor_in4, GPIO.LOW)
    pwm1.ChangeDutyCycle(85)
    pwm2.ChangeDutyCycle(55)
    print("🚗 Turning Left")

def move_right():
    GPIO.output(motor_in1, GPIO.HIGH)
    GPIO.output(motor_in2, GPIO.LOW)
    GPIO.output(motor_in3, GPIO.LOW)
    GPIO.output(motor_in4, GPIO.HIGH)
    pwm1.ChangeDutyCycle(50)
    pwm2.ChangeDutyCycle(70)
    print("🚗 Turning Right")

def move_forward():
    set_speed(move_speed)
    GPIO.output(motor_in1, GPIO.HIGH)
    GPIO.output(motor_in2, GPIO.LOW)
    GPIO.output(motor_in3, GPIO.HIGH)
    GPIO.output(motor_in4, GPIO.LOW)
    print("↗ Moving Forward")

def stop():
    set_speed(0)
    GPIO.output(motor_in1, GPIO.LOW)
    GPIO.output(motor_in2, GPIO.LOW)
    GPIO.output(motor_in3, GPIO.LOW)
    GPIO.output(motor_in4, GPIO.LOW)
    print("⏹ Stopping")

def turn_servo_left():
    print("🚗 Scanning Left")
    servo_pwm.ChangeDutyCycle(LEFT_DUTY)
    time.sleep(0.5)
    servo_pwm.ChangeDutyCycle(0)  # Stop signal

def turn_servo_right():
    print("🚗 Scanning Right")
    servo_pwm.ChangeDutyCycle(RIGHT_DUTY)
    time.sleep(0.5)
    servo_pwm.ChangeDutyCycle(0)  # Stop signal

def reset_servo():
    print("↔ Resetting Servo to Center")
    servo_pwm.ChangeDutyCycle(CENTER_DUTY)
    time.sleep(0.5)
    servo_pwm.ChangeDutyCycle(0)  # Stop signal

print("📱 Press 'q' to exit the live feed.")

try:
    while True:
        # Capture frame
        frame = picam2.capture_array()

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply Binary Threshold for Black Line on White Background
        _, threshold = cv2.threshold(gray, 115, 255, cv2.THRESH_BINARY_INV)

        # Find contours
        contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours (remove small noise)
        valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]

        if valid_contours:
            # Find contour closest to the bottom of the frame
            selected_contour = min(valid_contours, key=lambda c: cv2.boundingRect(c)[1])

            # Get bounding box
            x, y, w, h = cv2.boundingRect(selected_contour)

            # Draw bounding box
            cv2.rectangle(threshold, (x, y), (x + w, y + h), 0, 2)

            # Line center and frame center
            # Determine the new center of the frame (shifted by 20% upwards)
            original_center_y = threshold.shape[0] // 2  # Original vertical center
            shift_percentage = 0.2  # Shift by -20% (upwards)
            new_center_y = int(original_center_y + shift_percentage * threshold.shape[0])

            # Ensure it doesn't go beyond the frame boundaries
            new_center_y = max(min(new_center_y, threshold.shape[0] - 1), 0)

            # Line center and frame center (horizontal)
            line_center = x + w // 2
            frame_center = threshold.shape[1] // 2

            # Determine movement based on the new vertical center
            if line_center < frame_center - 90:
                move_right()  # Move left if the line is to the left of the frame center
                direction = "Turn Left"
            elif line_center > frame_center + 90:
                move_left()  # Move right if the line is to the right of the frame center
                direction = "Turn Right"
            else:
                move_forward()  # Go straight if the line is close to the frame center
                direction = "Go Straight"

            # Optionally, display the new center for debugging
            cv2.circle(threshold, (frame_center, new_center_y), 5, (0, 255, 0), 2)  # New center point

            # Display direction text on the frame
            cv2.putText(threshold, direction, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            # If no line detected, stop and start scanning
            stop()
            print("🚧 No line detected, scanning...")

            found_line = False  

            # Sweep servo left and right (Max 3 attempts each direction)
            for _ in range(3):
                turn_servo_left()
                frame = picam2.capture_array()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                _, threshold = cv2.threshold(gray, 115, 255, cv2.THRESH_BINARY_INV)
                contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if len(contours) > 0:
                    found_line = True
                    move_right()
                    break  

                turn_servo_right()
                frame = picam2.capture_array()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                _, threshold = cv2.threshold(gray, 115, 255, cv2.THRESH_BINARY_INV)
                contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if len(contours) > 0:
                    found_line = True
                    move_left()
                    break  

            reset_servo()

            if found_line:
                print("✅ Line found, resuming movement...")
            else:
                print("❌ No line found after scanning. Stopping robot.")
                time.sleep(5)
                break  

        # Show processed image
        if "DISPLAY" in os.environ:
            cv2.imshow("Line Detection", threshold)

        # Exit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("🚫 Stopping robot...")

except Exception as e:
    print("🛑 Error:", e)

# Cleanup
stop()
cv2.destroyAllWindows()
picam2.stop()
GPIO.cleanup()

