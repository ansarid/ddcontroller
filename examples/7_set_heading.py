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

import math
import time
from ddcontroller import DDRobot
import numpy as np


# Create robot object
robot = DDRobot(debug=True)

try:

    robot.left_wheel.motor.decay = 'SLOW'
    robot.right_wheel.motor.decay = 'SLOW'
    # While robot is running
    while robot.running:

        # Set robot's heading to pi/2 with a maximum angular velocity of 1 rad/s
        # robot.set_heading(math.pi/4, max_angular_velocity=6)
        robot.set_heading(np.deg2rad(205), max_angular_velocity=6)

        # Print the motion of the robot
        print(f"Target: {math.degrees(robot.target_heading)} degrees\tActual: {round(math.degrees(robot.get_heading()), 1)} degrees")

        # Run loop at 50Hz
        time.sleep(1/50)

except KeyboardInterrupt:
    print('Stopping...')

finally:
    # Clean up.
    robot.stop()
    print('Stopped.')