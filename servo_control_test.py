#test servo motor
import RPi.GPIO as GPIO
import time

# Configuration
SERVO_PIN = 18  # GPIO pin connected to the servo's signal wire (change if needed)
FREQUENCY = 50  # PWM frequency in Hz (standard for most servos)
CENTER_ANGLE = 0  # Starting and center position (user-defined angle)
MIN_ANGLE = -30  # Minimum allowed angle (degrees)
MAX_ANGLE = 30   # Maximum allowed angle (degrees)
STEP_DELAY = 0.02  # Delay between angle steps for slow movement (seconds)
STEP_SIZE = 1  # Angle increment per step for smooth motion

# Servo duty cycle calculation (assuming standard servo: 0° at ~5% duty, 180° at ~10% duty)
# Internal servo angles are mapped: user 0° -> servo 90°, user -30° -> servo 60°, user 30° -> servo 120°
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

# Main script
try:
    # Setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    pwm = GPIO.PWM(SERVO_PIN, FREQUENCY)
    pwm.start(0)  # Start PWM with 0 duty cycle

    # Always start at the center position
    print(f"Initializing servo to center position: {CENTER_ANGLE}°")
    set_angle(pwm, CENTER_ANGLE)
    time.sleep(1)  # Give time for servo to reach position

    # Test loop: slowly oscillate back and forth between MIN_ANGLE and MAX_ANGLE
    print("Starting test: slowly moving back and forth. Press Ctrl+C to stop.")
    current_angle = CENTER_ANGLE
    while True:
        # Slowly move to MAX_ANGLE
        for target in range(current_angle + STEP_SIZE, MAX_ANGLE + 1, STEP_SIZE):
            set_angle(pwm, target)
            time.sleep(STEP_DELAY)
        current_angle = MAX_ANGLE

        # Slowly move to MIN_ANGLE
        for target in range(current_angle - STEP_SIZE, MIN_ANGLE - 1, -STEP_SIZE):
            set_angle(pwm, target)
            time.sleep(STEP_DELAY)
        current_angle = MIN_ANGLE

except KeyboardInterrupt:
    print("Test stopped by user.")

finally:
    # Cleanup
    pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up.")
