import time as t
import board
import digitalio
from adafruit_simplemath import map_range
from circuitpython_gizmo import Gizmo
from robot import make_manny
from auton import doAutonA

BUTTON_BLANKING_PERIOD = 0.7
blanking_period_start = t.monotonic()
blanking_period_start_stick = t.monotonic()

# the Gizmo object provides access to the data that is held by the field
# management system and the gizmo system processor
gizmo = Gizmo()

robot = make_manny(gizmo)

prev_start_button = False

# start with the servo in the correct position so the boxes don't fall out
robot.assign_servo_to_angle(robot.box_servo, robot.servo_in_value)

# Configure the built-in LED pin as an output
builtin_led = digitalio.DigitalInOut(board.GP25)
builtin_led.direction = digitalio.Direction.OUTPUT

# Keep running forever
while True:

    # Toggle the built-in LED each time through the loop so we can see
    # that the program really is running.
    builtin_led.value = not builtin_led.value

    # Refreshes the information about axis and button states
    gizmo.refresh()

    # Mix right joystick axes to control both wheels
    speed = map_range(gizmo.axes.left_y, 0, 255, 1.0, -1.0)
    heading = map_range(gizmo.axes.right_x, 0, 255, 1.0, -1.0)
    robot.drive_base_arcade(speed, heading)

    # Control task motor with right trigger / shoulder button
    if gizmo.buttons.right_trigger:
        robot.drive_forklift_up()
    elif gizmo.buttons.right_shoulder:
        robot.drive_forklift_down()
    else:
        robot.stop_forklift()

    # logic that allows the servo move with a blanking period
    duration = t.monotonic() - blanking_period_start
    if gizmo.buttons.b and duration > BUTTON_BLANKING_PERIOD:
        blanking_period_start = t.monotonic()
        robot.eject_habitat()

    # activates autonomous task
    #if gizmo.buttons.left_stick and gizmo.buttons.right_stick:
    #    doAutonA(robot)

    # use the start button to switch if the filter is used or not
    if gizmo.buttons.start and not prev_start_button:
        if robot.use_filter:
            print(robot.use_filter)
        else:
            print(robot.use_filter)
    prev_start_button = gizmo.buttons.start
