import time
from ddcontroller import DDRobot

# Create robot object
robot = DDRobot()

try:

    # Create infinite loop
    while True:

        # Set robot's linear velocity to 0.2m/s and angular velocity to 1 rad/s
        robot.setMotion([0.2,1])

        # Print the motion of the robot
        print(robot.getMotion())

        # Run loop at 50Hz
        time.sleep(1/50)

except KeyboardInterrupt:
    print('Stopping...')

finally:
    # Clean up.
    robot.stop()
    print('Stopped.')