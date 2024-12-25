# DDController

[![documentation](https://img.shields.io/static/v1?label=docs&message=ddcontroller&color=informational)](https://ansarid.github.io/ddcontroller/)
[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](https://github.com/ansarid/ddcontroller/blob/master/LICENSE.md)
 
Python controller for differential drive robots.

## Install

### Manual Install

Install the AS5600L Encoder Library:

```bash
python3 -m pip install git+https://github.com/ansarid/as5600l
```

Install the DDController Library:

```bash
python3 -m pip install git+https://github.com/ansarid/ddcontroller
```

## Examples
### Set Robot Linear and Angular Velocity
```python
import time
from ddcontroller import DDRobot

# Create robot object
robot = DDRobot(debug=True)

try:

    # While robot is running
    while robot.running:

        # Set robot's linear velocity to 0.2m/s and angular velocity to 0.5 rad/s
        robot.set_motion([0.2, 0.5])

        # Get the motion of the robot
        motion = robot.get_motion()

        # Print the motion of the robot
        print(f"{round(motion[0], 3)} m/s\t{round(motion[1], 3)} rad/s")

        # Run loop at 50Hz
        time.sleep(1/50)

except KeyboardInterrupt:
    print('Stopping...')

finally:
    # Clean up.
    robot.stop()
    print('Stopped.')
```
### Command Robot to Travel to Global Positions
```python
import time
from ddcontroller import DDRobot

# Create robot object
robot = DDRobot(debug=True)

# Create list of Points
points = [
        [1,0],
        [1,1],
        [0,1],
        [0,0],
        ]

try:

    for point in points:

        # Print target location
        print(f"Headed to {point}.")

        # Set target location for navigation to (1,1)
        robot.go_to(point, tolerance=0.01, max_linear_velocity=0.3, max_angular_velocity=1)

        # Loop while robot is running and not at target location
        while robot.running and not robot.reached_target_position:

            # Get the robot's latest location
            x, y = robot.get_global_position()

            # Print the location of the robot
            print(f"Global Position: {round(x, 3)}, {round(y, 3)}", end="\r")

            # Run loop at 50Hz
            time.sleep(1/50)

        print('\nDone!')

except KeyboardInterrupt:
    print('Stopping...')

finally:
    # Clean up.
    robot.stop()
    print('Stopped.')
```
### More examples can be found in the [examples](https://github.com/ansarid/ddcontroller/tree/master/examples) folder.
