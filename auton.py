import time as t

start = t.monotonic()

def driveForward(time, motor_left, motor_right):
    start = t.monotonic()
    duration = t.monotonic() - start
    while duration < time:
        duration = t.monotonic() - start
        print(duration)
        motor_left.throttle = 1
        motor_right.throttle = 1

def driveTurnLeft(time, motor_left, motor_right):
    start = t.monotonic()
    duration = t.monotonic() - start
    while duration < time:
        duration = t.monotonic() - start
        print(duration)
        motor_left.throttle = -1
        motor_right.throttle = 1

def driveTurnRight(time, motor_left, motor_right):
    start = t.monotonic()
    duration = t.monotonic() - start
    while duration < time:
        duration = t.monotonic() - start
        print(duration)
        motor_left.throttle = 1
        motor_right.throttle = -1
