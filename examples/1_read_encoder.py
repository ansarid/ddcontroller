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
from as5600l import AS5600L

# Create encoder object
encoder = AS5600L(0x42)

try:

    # Create infinite loop
    while True:

        # Read the encoder position
        position = encoder.read_position()

        # Read the encoder angle
        angle = encoder.read_angle()

        # Print out the encoder position and angle
        print(f"Angle: {round(angle, 3)} radians\tPosition: {position}")

        # Run loop at 50Hz
        time.sleep(1/50)

except KeyboardInterrupt:
    pass

finally:
    pass
