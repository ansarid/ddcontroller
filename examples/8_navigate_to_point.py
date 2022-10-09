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
import math
from ddcontroller import DDRobot

# Create robot object
# robot = DDRobot(debug=True)
robot = DDRobot()

robot.heading_Kp = 80
robot.heading_Ki = 0
robot.heading_Kd = 0

start_time = time.monotonic()

try:

    # Set target location for navigation to (1,1)
    robot.go_to([0.5,0.5], tolerance=0.2, max_linear_velocity=0.2, max_angular_velocity=2)

    print('Time (s), X Position (m), Y Position (m), Heading (deg), X Target (m), Y Target (m), Heading Target (deg), XY Error (m), Heading Error (deg)')

    # Loop while robot is not at target location
    while not robot.reached_target_position:

        # Get the robot's latest location
        x,y = robot.get_global_position()

        # Print the location of the robot
        # print('Global Position: {}, {}'.format(round(x, 3), round(y, 3)))

        print(round(time.monotonic()-start_time, 3),',', round(x,3),',', round(y,3),',', round(robot.heading, 3),',', robot.target_position[0],',', robot.target_position[1],',', round(robot.target_heading,3),',', round(robot.position_error,3),',', round(robot.heading_error,3))

        # Run loop at 50Hz
        time.sleep(1/50)

except KeyboardInterrupt:
    print('Stopping...')

finally:
    # Clean up.
    robot.stop()
    print('Stopped.')