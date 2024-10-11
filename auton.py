import time as t

start = t.monotonic()

def driveForward(duration, robot):
    print(f" * Driving forward for {duration}s")
    start = t.monotonic()
    elapsed = t.monotonic() - start
    while elapsed < duration:
        elapsed = t.monotonic() - start
        #print(elapsed)
        robot.drive_base_arcade(speed=1.0, heading=0.0)

def driveTurnLeft(duration, robot):
    print(f" * Turning left for {duration}s")
    start = t.monotonic()
    elapsed = t.monotonic() - start
    while elapsed < duration:
        elapsed = t.monotonic() - start
        #print(elapsed)
        robot.drive_base_arcade(speed=0.0, heading=-1.0)

def driveTurnRight(duration, robot):
    print(f" * Turning right for {duration}s")
    start = t.monotonic()
    elapsed = t.monotonic() - start
    while elapsed < duration:
        elapsed = t.monotonic() - start
        #print(elapsed)
        robot.drive_base_arcade(speed=0.0, heading=1.0)

def driveForklift(duration, speed, robot):
    print(f" * Driving forklift with speed {speed} for {duration}s")
    start = t.monotonic()
    elapsed = t.monotonic() - start
    while elapsed < duration:
        elapsed = t.monotonic() - start
        #print(elapsed)
        robot.drive_forklift(speed)

def stop(robot):
    print(f" * Stopping base")
    robot.stop_base()

def doAutonA(robot):
    print("Running Auton A")
    driveForward(1.5, robot)
    stop(robot)
    # pickup ball
    # driveForklift(1, 1, robot)
    t.sleep(5)
    driveTurnRight(2, robot)
    stop(robot)
    driveForward(.75, robot)
    stop(robot)
