import RPi.GPIO as GPIO
import time

# Pin definitions
ENA = 19  # Left motor speed (PWM)
IN1 = 26  # Left motor direction 1
IN2 = 6   # Left motor direction 2
IN3 = 16  # Right motor direction 1
IN4 = 5   # Right motor direction 2
ENB = 13  # Right motor speed (PWM)

Servo = 18 #GPIO18 #12 #PWM0

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# PWM setup
pwm1 = GPIO.PWM(ENA, 200)  # Left motor PWM @ 200Hz
pwm2 = GPIO.PWM(ENB, 200)  # Right motor PWM @ 200Hz
pwm1.start(0)  # Start with 0% duty cycle (stopped)
pwm2.start(0)

# Function to move forward
def move_forward(speed, duration): #speed = duty (requires to  be determined for most effiecient)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)
    if duration:
        time.sleep(duration) #
        stop()

# Function to move backward
def move_backward(speed, duration):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)
    if duration:
        time.sleep(duration)
        stop()

# Function to turn left
def  turn_right(speed, duration):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)
    if duration:
        time.sleep(duration)
        stop()

# Function to turn right
def turn_left(speed, duration):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)
    if duration:
        time.sleep(duration)
        stop()

# Function to stop the robot
def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)

# Function to move for a set distance (approximate)
def move_distance(speed, distance_cm):
    time_per_cm=-0.000322801*(distance_cm)+0.031276  # Adjust this based on testing
    duration = distance_cm * time_per_cm
    move_forward(60, duration)
    # time_per_cm=-0.000322801(distance_cm)+0.031276


#Function to move for a set angle
def move_angle(speed, angle):
    angle_per_time= 100 # Adjust this based on testing # from 200 to 100 after testing on 12/3
    time_taken = ((angle))/angle_per_time  # Adjust this based on testing
    turn_right(speed, time_taken)

# Testing movements
try:
#    move_forward(80, 2)  # Move forward at 80% speed for 2 seconds
#    turn_left(80, 1)  # Turn left for 1 second
   # turn_right(80, 1)  # Turn right for 1 second (60,1 = 60deg)
   # stop()
   # time.sleep(5)
#    move_backward(70, 2)  # Move backward at 70% speed for 2 seconds
#    move_distance(60, 15)  # Move forward for 20 cm
#    stop()
#    move_angle(60,90) #180=300 @60 speed and angle_per_time =20 and time taken *0.1; 5 deg off @100
    stop()
finally:
    GPIO.cleanup()  # Reset GPIO when done

#distance
    #5cm 0.03657894737 --> 0.0332535885
    #10cm 0.0277113237 --> 0.0225295314 (0.02512042755)
    #15cm 0.0279115861 --> 0.0194731996 (0.02369239285)
    #20cm 0.02369239285
    #25cm 0.026309749 --> 0.0248205179 (0.025565133)
    #30cm 0.022492778 --> 0.0226263842 (0.022437831)

#angle
    #60,1
    #60 degrees

    #100 speed btw
    #60 gives 390 degrees
    #10 gives 112
    #9 gives 91 or 90 (ACCURATE)
    #18 gives 185 187
    #360 gives 380

    #when multiplied by 0.1,

    #95 inputted gives 90 degrees

    #50 gave 45 degrees

    #65 gives




    #now speed is 60

    #60 speed 150 gives 92

