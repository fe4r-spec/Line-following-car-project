import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import math
from picamera2 import Picamera2

# Motor pin configuration
MOTOR_PINS = {
    'IN1': 24, 'IN2': 23,
    'IN3': 27, 'IN4': 22,
    'ENA': 18, 'ENB': 12
}
PWM_FREQ = 100

# PID constants
KP, KI, KD = 0.5, 0.01, 0.1
pid_integral = 0
pid_last_error = 0

# Speeds
BASE_SPEED = 24
MAX_SPEED = 100
REVERSE_BIAS = 20       # additional speed on one side when reversing
MAX_REVERSE_TIME = 1.0  # seconds
FRAME_RATE = 30         # approximate

# Thresholds
BLACK_AREA_THRESHOLD = 100    # min contour area
MIN_DETECTED_PIXELS = 500     # for switching to shortcut
SHORTCUT_TOP_PIXELS = 100     # for slowdown
NO_LINE_THRESHOLD = 10        # frames before recovery

# Colors
COLOR_RANGES = {
    "blue":   ([0, 201, 27], [20, 255, 125]),
    "green":  ([40, 120, 70],  [80, 255, 255]),
    "yellow": ([20, 120, 70],  [40, 255, 255]),
    "red":    ([0, 120, 70],   [10, 255, 255], [170,120,70],[180,255,255])
}

SHORTCUT_COLORS = ["yellow", "green"]  

# Shape detection parameters
arrow_minHSV = np.array([100, 100, 50])
arrow_maxHSV = np.array([140, 255, 255])
final_shape_text = "-----"
shape_counter = 0

# GPIO setup
GPIO.setmode(GPIO.BCM)
for pin in MOTOR_PINS.values():
    GPIO.setup(pin, GPIO.OUT)
left_pwm  = GPIO.PWM(MOTOR_PINS['ENA'], PWM_FREQ)
right_pwm = GPIO.PWM(MOTOR_PINS['ENB'], PWM_FREQ)
left_pwm.start(0)
right_pwm.start(0)

def set_motors(ls, rs):
    """Set motor speeds with sign controlling direction pins."""
    ls = max(-MAX_SPEED, min(MAX_SPEED, ls))
    rs = max(-MAX_SPEED, min(MAX_SPEED, rs))
    GPIO.output(MOTOR_PINS['IN1'], ls>0); GPIO.output(MOTOR_PINS['IN2'], ls<0)
    GPIO.output(MOTOR_PINS['IN3'], rs>0); GPIO.output(MOTOR_PINS['IN4'], rs<0)
    left_pwm.ChangeDutyCycle(abs(ls))
    right_pwm.ChangeDutyCycle(abs(rs))

def process_frame(frame):
    """Returns (centroid, line_type, shortcut_ahead, vis_frame)."""
    h, w = frame.shape[:2]
    roi = frame[h//2:, :]  # bottom half for line following

    # 1) Black line detection
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, thr = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    cnts, _ = cv2.findContours(thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    black_cx = None
    if cnts:
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) > BLACK_AREA_THRESHOLD:
            m = cv2.moments(c)
            if m["m00"]:
                black_cx = int(m["m10"]/m["m00"]) - w//2

    # 2) Shortcut color detection
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    sc_mask = None
    for col, ranges in COLOR_RANGES.items():
        # build mask
        if col == "red":
            l1, u1, l2, u2 = [np.array(r) for r in ranges]
            mask = cv2.inRange(hsv, l1, u1) | cv2.inRange(hsv, l2, u2)
        else:
            l, u = [np.array(r) for r in ranges]
            mask = cv2.inRange(hsv, l, u)

        # find largest contour in that mask
        cnts2, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts2:
            c2 = max(cnts2, key=cv2.contourArea)
            if cv2.contourArea(c2) > BLACK_AREA_THRESHOLD:
                m2 = cv2.moments(c2)
                if m2["m00"]:
                    cx2 = int(m2["m10"]/m2["m00"]) - w//2
                    if col in SHORTCUT_COLORS:
                        sc_mask = (mask, cx2)

    # 3) Shortcut ahead detection: top slice
    shortcut_ahead = False
    if sc_mask:
        mask, _cx = sc_mask
        top_slice = mask[0:int(h/4), :]
        if np.count_nonzero(top_slice) > SHORTCUT_TOP_PIXELS:
            shortcut_ahead = True

    # 4) Decide active line
    if sc_mask and shortcut_ahead:
        return sc_mask[1], "shortcut", True, frame
    return black_cx, "black", False, frame

def detect_shape(cnt, frame, hsv_frame):
    shape = "unknown"
    peri = cv2.arcLength(cnt, True)
    area = cv2.contourArea(cnt)
 
    if area < 500 or len(cnt) < 3:
         return shape
 
    epsilon = 0.03 * peri if area > 1000 else 0.05 * peri
    approx = cv2.approxPolyDP(cnt, epsilon, True)
    num_vertices = len(approx)
    
    # First check for triangle (simplest shape)
    if num_vertices == 3:
        shape = "triangle"
        return shape
        
    # Check for arrow specifically if it's blue
    elif 4 <= num_vertices <= 8:
        # Get color information to confirm if it's blue
        maskHSV = cv2.inRange(hsv_frame, arrow_minHSV, arrow_maxHSV)
        kernel = np.ones((5, 5), np.uint8)
        maskHSV = cv2.morphologyEx(maskHSV, cv2.MORPH_CLOSE, kernel)
        maskHSV = cv2.morphologyEx(maskHSV, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        x, y, w, h = cv2.boundingRect(cnt)
        x, y = max(x - 10, 0), max(y - 10, 0)
        w = min(w + 20, frame.shape[1] - x)
        h = min(h + 20, frame.shape[0] - y)
        arrow_region = maskHSV[y:y + h, x:x + w]
        
        if arrow_region.size > 0:
            blurIm = cv2.GaussianBlur(arrow_region, (9, 9), 0)
            corners = cv2.goodFeaturesToTrack(blurIm, 2, 0.7, 15)

            if corners is not None and len(corners) >= 2:
                corners = np.int0(corners)
                x0, y0 = corners[0].ravel()
                x1, y1 = corners[1].ravel()
                x0, y0 = x0 + x, y0 + y
                x1, y1 = x1 + x, y1 + y

                cv2.circle(frame, (x0, y0), 5, (0, 0, 255), -1)
                cv2.circle(frame, (x1, y1), 5, (0, 0, 255), -1)

                am, bm = (x0 + x1) / 2, (y0 + y1) / 2
                cv2.circle(frame, (int(am), int(bm)), 3, (255, 0, 0), -1)

                (cx, cy), radius = cv2.minEnclosingCircle(cnt)
                cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 0, 255), 2)
                cv2.line(frame, (int(cx), int(cy)), (int(am), int(bm)), (255, 0, 0), 2)

                angle = math.degrees(math.atan2(bm - cy, am - cx))
                if -45 <= angle < 45:
                    return "arrow (right)"
                elif 45 <= angle < 135:
                    return "arrow (down)"
                elif -180 <= angle <= -135 or 135 <= angle <= 180:
                    return "arrow (left)"
                elif -135 < angle < -45:
                    return "arrow (up)"
    
    # Continue with regular shape detection
    if num_vertices == 4:
        x, y, w, h = cv2.boundingRect(approx)
        ar = w / float(h)
        shape = "square" if 0.95 <= ar <= 1.05 else "rectangle"
    elif num_vertices == 5:
        shape = "pentagon"
    elif num_vertices == 6:
        shape = "hexagon"
    elif num_vertices > 6:
        circularity = (4 * math.pi * area) / (peri * peri)
        shape = "full circle" if circularity > 0.8 else "partial circle"

    return shape

def main():
    global pid_integral, pid_last_error, final_shape_text, shape_counter
    
    picam = Picamera2()
    cfg = picam.create_preview_configuration(main={"size":(320,240),"format":"RGB888"})
    picam.configure(cfg)
    picam.start()

    state = "follow"
    last_centroid = None
    lost_frames = 0
    reverse_start = None
    reverse_dir = None

    dt_target = 1/FRAME_RATE
    prev = time.time()

    while True:
        frame = picam.capture_array()
        
        # LINE FOLLOWING PROCESSING (uses bottom half of frame)
        c, line_type, ahead, vis = process_frame(frame)

        if c is not None:
            # got line
            last_centroid = c
            lost_frames = 0
            state = "follow"
        else:
            lost_frames += 1
            if lost_frames > NO_LINE_THRESHOLD and state == "follow":
                # enter reverse state
                state = "reverse"
                reverse_start = time.time()
                # pick direction based on last_centroid
                if last_centroid is not None and last_centroid < 0:
                    reverse_dir = "left"
                else:
                    reverse_dir = "right"

        # SHAPE DETECTION PROCESSING (uses full frame)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        edges = cv2.Canny(thresh, 120, 100)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) > 500:  # Filter small contours
                shape = detect_shape(cnt, frame, hsv_frame)
                # Shape persistence logic
                shape_text = shape if shape.startswith("arrow") else shape
                if final_shape_text != shape_text:
                    shape_counter += 1
                else:
                    shape_counter = 0
                if shape_counter >= 5:
                    final_shape_text = shape_text

                print(f"Detected shape: {shape}")
                cv2.drawContours(frame, [cnt], -1, (0, 0, 255), 2)
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.putText(frame, final_shape_text, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # STATE MACHINE FOR LINE FOLLOWING
        if state == "follow" and c is not None:
            # PID line following
            error = c
            pid_integral += error
            derivative = error - pid_last_error
            pid_last_error = error
            corr = KP*error + KI*pid_integral + KD*derivative

            ls = BASE_SPEED - corr
            rs = BASE_SPEED + corr
            set_motors(ls, rs)

        elif state == "reverse":
            elapsed = time.time() - reverse_start
            if elapsed > MAX_REVERSE_TIME:
                state = "follow"
                pid_integral = pid_last_error = 0
            else:
                if reverse_dir == "left":
                    set_motors(-(BASE_SPEED+REVERSE_BIAS), -BASE_SPEED)
                else:
                    set_motors(-BASE_SPEED, -(BASE_SPEED+REVERSE_BIAS))

        # display
        cv2.putText(frame, f"State:{state}  Line:{line_type}", (10,20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        cv2.imshow("Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # frame rate cap
        now = time.time()
        dt = now - prev
        if dt < dt_target:
            time.sleep(dt_target - dt)
        prev = now

    # cleanup
    set_motors(0,0)
    left_pwm.stop()
    right_pwm.stop()
    GPIO.cleanup()
    picam.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()