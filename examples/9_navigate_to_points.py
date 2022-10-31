#!/usr/bin/env python3

"""
This file is part of the ddcontroller library (https://github.com/ansarid/ddcontroller).
Copyright (C) 2022  Daniyal Ansari

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

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
        robot.go_to(point, tolerance=0.1, max_linear_velocity=0.3, max_angular_velocity=2)

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