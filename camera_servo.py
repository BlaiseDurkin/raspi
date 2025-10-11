import RPi.GPIO as GPIO
import time
import cv2
import mediapipe as mp

# Configuration
SERVO_PIN = 18  # GPIO pin connected to the servo's signal wire
FREQUENCY = 50  # PWM frequency in Hz
CENTER_ANGLE = -90  # Starting and center position (user-defined angle)
MIN_ANGLE = CENTER_ANGLE - 45  # Minimum allowed angle (degrees)
MAX_ANGLE = CENTER_ANGLE + 70  # Maximum allowed angle (degrees)
STEP_DELAY = 0.02  # Delay between angle steps for slow movement (seconds)
STEP_SIZE = 10  # Angle increment per step for smooth motion

# Initialize servo
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm = GPIO.PWM(SERVO_PIN, FREQUENCY)
pwm.start(0)  # Start once here, with 0 duty (inactive)

# Servo duty cycle calculation (restored original formula)
def calculate_duty_cycle(user_angle):
    servo_angle = 90 + user_angle  # Map user angle to servo angle
    duty = 5 + (servo_angle / 36)  # Original formula
    return duty
# Function to set the servo angle with bounds checking
def set_angle(pwm, angle):
    if angle < MIN_ANGLE:
        angle = MIN_ANGLE
        print(f"Angle clamped to minimum: {MIN_ANGLE}°")
    elif angle > MAX_ANGLE:
        angle = MAX_ANGLE
        print(f"Angle clamped to maximum: {MAX_ANGLE}°")
    
    duty = calculate_duty_cycle(angle)
    pwm.ChangeDutyCycle(duty)
    return angle

def turn_left(current_angle):
    print('turning to the TV')
    # Always start at the center position
    print(f"Initializing servo to center position: {CENTER_ANGLE}°")
    set_angle(pwm, CENTER_ANGLE)
    time.sleep(1)  # Give time for servo to reach position
    
    # Slowly move to MIN_ANGLE
    current_angle = CENTER_ANGLE
    while current_angle > MIN_ANGLE:
        current_angle -= STEP_SIZE
        if current_angle < MIN_ANGLE:
            current_angle = MIN_ANGLE
        set_angle(pwm, current_angle)
        print('target: ', current_angle)
        time.sleep(STEP_DELAY)
    
    pwm.ChangeDutyCycle(0)  # "Stop" signal without pwm.stop()
    return current_angle
    
def turn_right(current_angle):
    print('turning to the backyard')
    # Always start at the center position
    print(f"Initializing servo to center position: {CENTER_ANGLE}°")
    set_angle(pwm, CENTER_ANGLE)
    time.sleep(1)  # Give time for servo to reach position

    # Slowly move to MAX_ANGLE
    current_angle = CENTER_ANGLE
    while current_angle < MAX_ANGLE:
        current_angle += STEP_SIZE
        if current_angle > MAX_ANGLE:
            current_angle = MAX_ANGLE
        set_angle(pwm, current_angle)
        print('target: ', current_angle)
        time.sleep(STEP_DELAY)
    
    pwm.ChangeDutyCycle(0)  # "Stop" signal without pwm.stop()
    return current_angle
    

def turn_center():
    print('turning to the middle')
    set_angle(pwm, CENTER_ANGLE)
    time.sleep(1)  # Give time for servo to reach position
    pwm.ChangeDutyCycle(0)  # "Stop" signal without pwm.stop()

# Initialize video capture
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera")
    exit()

# Initialize MediaPipe Pose
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
pose = mp_pose.Pose(
    static_image_mode=False,
    model_complexity=1,
    enable_segmentation=False,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

"""
# Get frame dimensions
ret, frame = cap.read()
if not ret:
    print("Failed to capture frame")
    cap.release()
    exit()

height, width, _ = frame.shape
print(f"Window Height: {height}")
print(f"Window Width: {width}")

# Calculate h1 as 33% of the height
h1 = int(0.33 * height)
"""

# Create OpenCV window
cv2.namedWindow("Pose", cv2.WINDOW_NORMAL)

try:
    # Always start at the center position
    print(f"Initializing servo to center position: {CENTER_ANGLE}°")
    current_angle = set_angle(pwm, CENTER_ANGLE)
    time.sleep(1)  # Give time for servo to reach position
    pwm.ChangeDutyCycle(0)  # "Stop" initially

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Convert frame to RGB for MediaPipe
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = pose.process(rgb)

        # Draw pose landmarks if detected
        if results.pose_landmarks:
            mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

        # Draw horizontal line
        #cv2.line(frame, (0, h1), (width, h1), (0, 0, 255), 2)

        # Display the frame
        cv2.imshow("Pose", frame)

        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Exiting program.")
            break
        elif key == ord('a'):
            current_angle = turn_left(current_angle)
            print(f"Current angle: {current_angle}°")
        elif key == ord('d'):
            current_angle = turn_right(current_angle)
            print(f"Current angle: {current_angle}°")

except KeyboardInterrupt:
    print("Program stopped by Ctrl+C.")
finally:
    pwm.ChangeDutyCycle(0)
    pwm.stop()  # Safe to stop here at full cleanup
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()
    print("Resources cleaned up.")