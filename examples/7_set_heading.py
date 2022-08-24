import math
import time
from ddcontroller import DDRobot

# Create robot object
robot = DDRobot()

try:

    # Create infinite loop
    while True:

        # Set robot's heading to pi/2
        robot.setHeading(math.pi/2)

        # Print the motion of the robot
        print(robot.getHeading())

        # Run loop at 50Hz
        time.sleep(1/50)

except KeyboardInterrupt:
    print('Stopping...')

finally:
    # Clean up.
    robot.stop()
    print('Stopped.')