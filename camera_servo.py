import cv2
import mediapipe as mp
import RPi.GPIO as GPIO
import time

# Configuration
SERVO_PIN = 18  # GPIO pin connected to the servo's signal wire
FREQUENCY = 50  # PWM frequency in Hz
CENTER_ANGLE = -90  # Starting and center position (user-defined angle)
MIN_ANGLE = CENTER_ANGLE - 45  # Minimum allowed angle (degrees)
MAX_ANGLE = CENTER_ANGLE + 60  # Maximum allowed angle (degrees)
STEP_DELAY = 0.02  # Delay between angle steps for slow movement (seconds)
STEP_SIZE = 10  # Angle increment per step for smooth motion


# Initialize servo
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm = GPIO.PWM(SERVO_PIN, FREQUENCY)


# Servo duty cycle calculation
def calculate_duty_cycle(user_angle):
    servo_angle = 90 + user_angle  # Map user angle (-30 to 30) to servo angle (60 to 120)
    duty = 5 + (servo_angle / 36)  # Formula for duty cycle
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

def turn_left(ang):
    print('turning to the TV')
    duty = calculate_duty_cycle(ang)
    # Slowly move to MIN_ANGLE
    pwm.start(duty)  # Start PWM with 0 duty cycle

    # Always start at the center position
    print(f"Initializing servo to center position: {CENTER_ANGLE}°")
    set_angle(pwm, CENTER_ANGLE)
    time.sleep(1)  # Give time for servo to reach position
    # Test loop: slowly oscillate back and forth between MIN_ANGLE and MAX_ANGLE
    current_angle = CENTER_ANGLE
    for target in range(current_angle - STEP_SIZE, MIN_ANGLE - 1, -STEP_SIZE):
        set_angle(pwm, target)
        print('target: ',target)
        time.sleep(STEP_DELAY)
    current_angle = MIN_ANGLE
    pwm.stop()
    return current_angle
    
def turn_right(ang):
    print('turning to the backyard')
    duty = calculate_duty_cycle(ang)
    pwm.start(duty)  # Start PWM with 0 duty cycle
    # Always start at the center position
    print(f"Initializing servo to center position: {CENTER_ANGLE}°")
    set_angle(pwm, CENTER_ANGLE)
    time.sleep(1)  # Give time for servo to reach position

    # Test loop: slowly oscillate back and forth between MIN_ANGLE and MAX_ANGLE
    current_angle = CENTER_ANGLE
    for target in range(current_angle + STEP_SIZE, MAX_ANGLE + 1, STEP_SIZE):
        set_angle(pwm, target)
        print('target: ',target)
        time.sleep(STEP_DELAY)
    current_angle = MAX_ANGLE
    pwm.stop()
    #turn off pin
    return current_angle
    

def turn_center():
    print('turning to the middle')
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
    pwm.start(0)  # Start PWM with 0 duty cycle

    # Always start at the center position
    print(f"Initializing servo to center position: {CENTER_ANGLE}°")
    current_angle = set_angle(pwm, CENTER_ANGLE)
    time.sleep(1)  # Give time for servo to reach position

    # Test loop: slowly oscillate back and forth between MIN_ANGLE and MAX_ANGLE
    #current_angle = CENTER_ANGLE
    pwm.stop()
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
    
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()
    print("Resources cleaned up.")