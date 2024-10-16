import time as t

start = t.monotonic()

def driveForward(duration, robot):
    """Makes robot drive forward.

    Args:
        duration (_type_): How long the robot drives forward for.
        robot (_type_): The robot object.
    """
    print(f" * Driving forward for {duration}s")
    start = t.monotonic()
    elapsed = t.monotonic() - start
    while elapsed < duration:
        elapsed = t.monotonic() - start
        #print(elapsed)
        robot.drive_base_arcade(speed=1.0, heading=0.0)

def driveTurnLeft(duration, robot): 
    """Turns robot left.

    Args:
        duration (_type_): How long the robot is turned for.
        robot (_type_): The robot class.
    """
    print(f" * Turning left for {duration}s")
    start = t.monotonic()
    elapsed = t.monotonic() - start
    while elapsed < duration:
        elapsed = t.monotonic() - start
        #print(elapsed)
        robot.drive_base_arcade(speed=0.0, heading=-1.0)

def driveTurnRight(duration, robot):
    """Turns the robot right.

    Args:
        duration (_type_): How long the robot turns for.
        robot (_type_): The robot class.
    """
    print(f" * Turning right for {duration}s")
    start = t.monotonic()
    elapsed = t.monotonic() - start
    while elapsed < duration:
        elapsed = t.monotonic() - start
        #print(elapsed)
        robot.drive_base_arcade(speed=0.0, heading=1.0)

def driveForklift(duration, speed, robot):
    """Moves forklift.

    Args:
        duration (_type_): How long the forklift moves for.
        speed (_type_): The speed that the forklift moves at.
        robot (_type_): The robot class.
    """
    print(f" * Driving forklift with speed {speed} for {duration}s")
    start = t.monotonic()
    elapsed = t.monotonic() - start
    while elapsed < duration:
        elapsed = t.monotonic() - start
        #print(elapsed)
        robot.drive_forklift(speed)

def stop(robot):
    """Stops the robot.

    Args:
        robot (_type_): The robot object.
    """
    print(f" * Stopping base")
    robot.stop_base()

def doAutonA(robot):
    """Runs autonomous task.

    Args:
        robot (_type_): The robot object.
    """
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
