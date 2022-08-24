import time
from ddcontroller import DDRobot

# Create robot object
robot = DDRobot()

# Create Path with list of Points
path = [
        [1,0],
        [1,1],
        [0,1],
        [0,0],
        ]

try:

    # Set path for robot to navigate
    robot.followPath(path)

    # Loop while robot is in motion
    while robot.isMoving():

        # Get the robot's latest location
        x,y = robot.getGlobalPosition()

        # Print the location of the robot
        print('Global Position: {}, {}'.format(round(x, 3), round(y, 3)))

        # Run loop at 50Hz
        time.sleep(1/50)

except KeyboardInterrupt:
    print('Stopping...')

finally:
    # Clean up.
    robot.stop()
    print('Stopped.')