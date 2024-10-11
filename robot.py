import pwmio
import time as t
from adafruit_motor import servo
from adafruit_simplemath import constrain
from filter import Filter

def log_hz(stamp, period, msg):
    duration = t.monotonic() - stamp
    if duration >= period:
        stamp = t.monotonic()
        print(msg)
    return stamp

class Robot:
    def __init__(self):
        self.left_motor = None
        self.right_motor = None
        self.forklift_motor = None
        self.collection_motor = None
        self.box_servo = None

        self.pwm_freq = 50  # Hertz
        self.min_pulse = 1000  # milliseconds
        self.max_pulse = 2000  # milliseconds
        self.servo_range = 90  # degrees

        self.SERVO_IN_VALUE = 0
        self.SERVO_OUT_VALUE = 90

        self.time_stamp = t.monotonic()

        self.speed_filter = Filter()
        self.heading_filter = Filter()

    def _make_motor(self, addr):
        motor = servo.ContinuousServo(
            pwmio.PWMOut(addr, frequency=self.pwm_freq),
            min_pulse=self.min_pulse,
            max_pulse=self.max_pulse,
        )
        return motor

    def _make_servo(self, addr):
        a_servo = servo.Servo(
            pwmio.PWMOut(addr, frequency=self.pwm_freq),
            actuation_range=self.servo_range,
            min_pulse=self.min_pulse,
            max_pulse=self.max_pulse,
        )
        return a_servo

    def drive_base_arcade(self, speed, heading):
        if not self.left_motor or not self.right_motor:
            raise Exception("Robot not initialized")

        speed_filtered = self.speed_filter.filter(speed)
        heading_filtered = self.heading_filter.filter(heading)

        left_speed = constrain(speed_filtered + heading_filtered, -1.0, 1.0)
        right_speed = constrain(speed_filtered - heading_filtered, -1.0, 1.0)
        self.left_motor.throttle = -1*left_speed
        self.right_motor.throttle = -1*right_speed

    def stop_base(self):
        if not self.left_motor or not self.right_motor:
            raise Exception("Robot not initialized")

        self.speed_filter.reset()
        self.heading_filter.reset()
        self.left_motor.throttle = 0.0
        self.right_motor.throttle = 0.0

    def drive_forklift(self, speed):
        if not self.forklift_motor:
            raise Exception("Robot not initialized")
        speed_filtered = constrain(speed, -1.0, 1.0)
        self.forklift_motor.throttle = speed_filtered

    def drive_forklift_up(self):
        self.drive_forklift(1.0)

    def drive_forklift_down(self):
        self.drive_forklift(-1.0)

    def stop_forklift(self):
        self.drive_forklift(0.0)

    def eject_habitat(self):
        self.box_servo.angle = self.SERVO_OUT_VALUE
        t.sleep(0.3)
        self.box_servo.angle = self.SERVO_IN_VALUE


def make_manny(gizmo):
    robot = Robot()
    robot.left_motor = robot._make_motor(gizmo.MOTOR_1)
    robot.right_motor = robot._make_motor(gizmo.MOTOR_3)
    robot.forklift_motor = robot._make_motor(gizmo.MOTOR_2)
    # robot.collection_motor = robot._make_motor(gizmo.MOTOR_0)
    robot.box_servo = robot._make_servo(gizmo.SERVO_1)

    return robot
