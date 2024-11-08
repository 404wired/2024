import pwmio
import time as t
from adafruit_motor import servo
from adafruit_simplemath import constrain
from filter import Filter

class Robot:
    """Provides functions to manipulate robot.
    """
    def __init__(self):
        """Variables for the motor and servos.
        """
        # assigns values for motor and servo variables
        self.left_motor = None
        self.right_motor = None
        self.forklift_motor = None
        self.collection_motor = None
        self.box_servo = None

        self.pwm_freq = 50  # Hertz
        self.min_pulse = 1000  # milliseconds
        self.max_pulse = 2000  # milliseconds
        self.servo_range = 120  # degrees

        self.servo_in_value = 45 # usually 0
        self.servo_out_value = 0 # usually 120

        self.is_servo_holding_modules = False

        self.speed_filter = Filter()
        self.heading_filter = Filter()

        self.use_filter = True

    def _make_motor(self, addr):
        """Makes a motor.

        Args:
            addr (_type_): Assigns a port to the motor.

        Returns:
            _type_: Created motor.
        """
        # sets parameters for motor
        motor = servo.ContinuousServo(
            pwmio.PWMOut(addr, frequency=self.pwm_freq),
            min_pulse=self.min_pulse,
            max_pulse=self.max_pulse,
        )
        return motor

    def _make_servo(self, addr):
        """Makes a servo.

        Args:
            addr (_type_): Assigns a port to the servo.

        Returns:
            _type_: Created servo.
        """
        # sets parameters for the servo
        a_servo = servo.Servo(
            pwmio.PWMOut(addr, frequency=self.pwm_freq),
            actuation_range=self.servo_range,
            min_pulse=self.min_pulse,
            max_pulse=self.max_pulse,
        )
        return a_servo

    def drive_base_arcade(self, speed, heading):
        """Drive's the robot.

        Args:
            speed (_type_): Sets speed.
            heading (_type_): Sets heading.

        Raises:
            Exception: Prevents error if motor is not created.
        """
        if not self.left_motor or not self.right_motor:
            raise Exception("Robot not initialized")

        if self.use_filter:
            # Filtering speed and heading
            speed_filtered = self.speed_filter.filter(speed)
            heading_filtered = self.heading_filter.filter(heading)

            # constrain the speed values
            left_speed = constrain(speed_filtered + heading_filtered, -1.0, 1.0)
            right_speed = constrain(speed_filtered - heading_filtered, -1.0, 1.0)
        else:
            left_speed = constrain(speed + heading, -1.0, 1.0)
            right_speed = constrain(speed - heading, -1.0, 1.0)

        # assigns speed to motors
        self.left_motor.throttle = -1*left_speed
        self.right_motor.throttle = -1*right_speed

    def stop_base(self):
        """Stops the robot.

        Raises:
            Exception: Prevents error if motor is not made.
        """
        if not self.left_motor or not self.right_motor:
            raise Exception("Robot not initialized")

        # Resets memory values in speed filter
        self.speed_filter.reset()
        self.heading_filter.reset()
        # assigns motors throttle 0 to stop robot
        self.left_motor.throttle = 0.0
        self.right_motor.throttle = 0.0

    def drive_forklift(self, speed):
        """Makes the forklift move.

        Args:
            speed (_type_): Speed to move the forklift.

        Raises:
            Exception: Prevents error if motor is not made.
        """
        if not self.forklift_motor:
            raise Exception("Robot not initialized")

        forklift_speed = constrain(speed, -1.0, 1.0)
        self.forklift_motor.throttle = forklift_speed

    def drive_forklift_up(self):
        """Drive forklift up.
        """
        self.drive_forklift(1.0)

    def drive_forklift_down(self):
        """Drive forklift down.
        """
        self.drive_forklift(-1.0)

    def stop_forklift(self):
        """Stops forklift.
        """
        self.drive_forklift(0.0)

    def assign_servo_to_angle(self, servo, angle):

        # ensure servo angle is within servo opperating range
        if angle > self.servo_range or angle < 0:
            raise Exception(f"Provided servo angle must be within 0-{self.servo_range}")

        servo.angle = angle

    def eject_habitat(self):
        """Makes servo move to eject habitat modules.
        """
        # old code that dispenses the modules
        # moves servo
        # self.box_servo.angle = self.servo_out_value
        # t.sleep(0.3)
        # self.box_servo.angle = self.servo_in_value

        if self.is_servo_holding_modules:
            self.assign_servo_to_angle(self.box_servo, self.servo_out_value)
            self.is_servo_holding_modules = False
        else:
            self.assign_servo_to_angle(self.box_servo, self.servo_in_value)
            self.is_servo_holding_modules = True

def make_manny(gizmo):
    """Assigns ports for motors and servo.

    Args:
        gizmo (_type_): The area of the port that motors and servos are assigned to.

    Returns:
        _type_: Makes robot.
    """
    # assigns ports for motors and servo
    robot = Robot()
    robot.left_motor = robot._make_motor(gizmo.MOTOR_2)
    robot.right_motor = robot._make_motor(gizmo.MOTOR_1)
    robot.forklift_motor = robot._make_motor(gizmo.MOTOR_3)
    # robot.collection_motor = robot._make_motor(gizmo.MOTOR_0)
    robot.box_servo = robot._make_servo(gizmo.SERVO_1)

    return robot
