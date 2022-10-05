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

# Create Robot object
robot = DDRobot(debug=True)

goals = [
        [1, 0],
        [1,-1],
        [0,-1],
        [0, 0],
        ]

try:

    while True:

        for goal in goals:
            print('Headed to target:', goal)
            error = robot.go_to(goal, tolerance=0.2, max_linear_velocity=0.2, max_angular_velocity=0.75)
            print('Error:', error)
            print('Reached target:', goal, error)

    # Run loop at 50Hz
    # time.sleep(1/50)

except KeyboardInterrupt:
    print('Stopping...')

finally:
    # Clean up.
    robot.stop()
    print('Stopped.')

