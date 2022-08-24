import time
from ddcontroller import DDRobot

# Create robot object
robot = DDRobot()

try:

    # Set target location for navigation to (1,1)
    robot.goTo([1,1])

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