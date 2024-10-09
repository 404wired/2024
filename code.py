import time as t
import board
import pwmio
import digitalio
from adafruit_motor import servo
from adafruit_simplemath import map_range, constrain
from circuitpython_gizmo import Gizmo
from auton import driveForward, driveTurnLeft, driveTurnRight

BUTTON_BLANKING_PERIOD = 0.7
blanking_period_start = t.monotonic()

# the Gizmo object provides access to the data that is held by the field
# management system and the gizmo system processor
gizmo = Gizmo()

pwm_freq = 50 # Hertz
min_pulse = 1000 # milliseconds
max_pulse = 2000 # milliseconds
servo_range = 90  # degrees

# Configure the motors & servos for the ports they are connected to
motor_left = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_1, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
motor_right = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_3, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
motor_task = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_2, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
servo_task = servo.Servo(
    pwmio.PWMOut(gizmo.SERVO_1, frequency=pwm_freq),
    actuation_range=servo_range,
    min_pulse=min_pulse,
    max_pulse=max_pulse
)

# Configure the built-in LED pin as an output
builtin_led = digitalio.DigitalInOut(board.GP25)
builtin_led.direction = digitalio.Direction.OUTPUT

TANK_MODE = 0
ARCADE_MODE = 1

mode = TANK_MODE

prev_start_button = False

# Keep running forever
while True:

    # Toggle the built-in LED each time through the loop so we can see
    # that the program really is running.
    builtin_led.value = not builtin_led.value

    # Refreshes the information about axis and button states
    gizmo.refresh()

    # If the start button was pressed, switch control mode
    if gizmo.buttons.start and not prev_start_button:
        if mode == TANK_MODE:
            mode = ARCADE_MODE
        elif mode == ARCADE_MODE:
            mode = TANK_MODE
    prev_start_button = gizmo.buttons.start

    if mode == TANK_MODE:
        # Convert gamepad axis positions (0 - 255) to motor speeds (-1.0 - 1.0)
        # looks wrong but correct
        motor_left.throttle = map_range(gizmo.axes.left_y, 0, 255, -1.0, 1.0)
        motor_right.throttle = map_range(gizmo.axes.right_y, 0, 255, -1.0, 1.0)
    elif mode == ARCADE_MODE:
        # Mix right joystick axes to control both wheels
        speed = map_range(gizmo.axes.left_y, 0, 255, -1.0, 1.0)
        steering = map_range(gizmo.axes.right_x, 0, 255, -1.0, 1.0)
        motor_left.throttle = constrain(speed - steering, -1.0, 1.0)
        motor_right.throttle = constrain(speed + steering, -1.0, 1.0)

    # Control task motor with right trigger / shoulder button
    if gizmo.buttons.right_trigger:
        motor_task.throttle = 1.0
    elif gizmo.buttons.right_shoulder:
        motor_task.throttle = -1.0
    else:
        motor_task.throttle = 0.0

    # Control task servo with left trigger / shoulder button
    SERVO_IN_VALUE = 0
    SERVO_OUT_VALUE = 90

    duration = t.monotonic() - blanking_period_start
    if gizmo.buttons.b and duration > BUTTON_BLANKING_PERIOD:
        blanking_period_start = t.monotonic()
        servo_task.angle = SERVO_OUT_VALUE
        t.sleep(.3)
        servo_task.angle = SERVO_IN_VALUE

    if gizmo.buttons.y:
       print("driving forward")
       driveForward(5, motor_left, motor_right)
       print("sleeping...")
       t.sleep(1)
       print("turning left")
       driveTurnLeft(5, motor_left, motor_right)
       print("sleeping...")
       t.sleep(1)
       print("turning left")
       driveTurnRight(5, motor_left, motor_right)
       print("auton finished")
