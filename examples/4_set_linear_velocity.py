import time
from ddcontroller import DDRobot

# Create Robot object
robot = DDRobot()

try:

    # Create infinite loop
    while True:

        # Set robot's linear velocity to 0.2 m/s
        robot.setLinearVelocity(0.2)

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