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
        self.left_motor = None
        self.right_motor = None
        self.forklift_motor = None
        self.collection_motor = None
        self.box_servo = None

        self.pwm_freq = 50  # Hertz
        self.min_pulse = 1000  # milliseconds
        self.max_pulse = 2000  # milliseconds
        self.servo_range = 120  # degrees

        self.SERVO_IN_VALUE = 0
        self.SERVO_OUT_VALUE = 120

        self.speed_filter = Filter()
        self.heading_filter = Filter()

    def _make_motor(self, addr):
        """Makes a motor.

        Args:
            addr (_type_): A port to use for motor.

        Returns:
            _type_: Created motor.
        """
        motor = servo.ContinuousServo(
            pwmio.PWMOut(addr, frequency=self.pwm_freq),
            min_pulse=self.min_pulse,
            max_pulse=self.max_pulse,
        )
        return motor

    def _make_servo(self, addr): 
        """Makes a servo.

        Args:
            addr (_type_): Provides a port for the servo.

        Returns:
            _type_: Created servo.
        """
        a_servo = servo.Servo(
            pwmio.PWMOut(addr, frequency=self.pwm_freq),
            actuation_range=self.servo_range,
            min_pulse=self.min_pulse,
            max_pulse=self.max_pulse,
        )
        return a_servo

    def drive_base_arcade(self, speed, heading):
        """Sets drive mode to arcade.

        Args:
            speed (_type_): Sets speed.
            heading (_type_): Sets heading.

        Raises:
            Exception: Prevents error if motor is not made. 
        """
        if not self.left_motor or not self.right_motor:
            raise Exception("Robot not initialized")

        speed_filtered = self.speed_filter.filter(speed)
        heading_filtered = self.heading_filter.filter(heading)

        left_speed = constrain(speed_filtered + heading_filtered, -1.0, 1.0)
        right_speed = constrain(speed_filtered - heading_filtered, -1.0, 1.0)
        self.left_motor.throttle = -1*left_speed
        self.right_motor.throttle = -1*right_speed

    def stop_base(self):
        """Stops the robot.

        Raises:
            Exception: Prevents error if motor is not made.
        """
        if not self.left_motor or not self.right_motor:
            raise Exception("Robot not initialized")

        self.speed_filter.reset()
        self.heading_filter.reset()
        self.left_motor.throttle = 0.0
        self.right_motor.throttle = 0.0

    def drive_forklift(self, speed):
        """Makes the forklift move.

        Args:
            speed (_type_): Gives speed to the forklift. 

        Raises:
            Exception: Does not allow the forklift to move if it is not made.
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

    def eject_habitat(self):
        """Makes servo move to eject habitat modules.
        """
        self.box_servo.angle = self.SERVO_OUT_VALUE
        t.sleep(0.3)
        self.box_servo.angle = self.SERVO_IN_VALUE


def make_manny(gizmo):
    """Assigns ports for motors and servo.

    Args:
        gizmo (_type_): The area of the port that motors and servos are assigned to.

    Returns:
        _type_: Makes robot.
    """
    robot = Robot()
    robot.left_motor = robot._make_motor(gizmo.MOTOR_1)
    robot.right_motor = robot._make_motor(gizmo.MOTOR_3)
    robot.forklift_motor = robot._make_motor(gizmo.MOTOR_2)
    # robot.collection_motor = robot._make_motor(gizmo.MOTOR_0)
    robot.box_servo = robot._make_servo(gizmo.SERVO_1)

    return robot
