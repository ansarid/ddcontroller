#!/usr/bin/python3

"""
This file is part of the SCUTTLEPy library (https://github.com/ansarid/scuttlepy).
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
from .utils.gamepad import Gamepad
from scuttlepy import SCUTTLE

# Create gamepad object
gamepad = Gamepad()

# Create SCUTTLE Object
scuttle = SCUTTLE()

try:

    # Create infinite loop
    while True:

        # Get joystick x and y values and scale values to -1 to 1
        joystickX, joystickY = [((-2/255)*gamepad.axes['LEFT_X'])+1,
                                ((-2/255)*gamepad.axes['LEFT_Y'])+1
                                ]

        motion = [
                  # Mutiply joystick y axis by SCUTTLE max linear velocity to get linear velocity
                  joystickY*scuttle.maxVelocity,

                  # Mutiply joystick x axis by SCUTTLE max angular velocity to get angular velocity
                  joystickX*scuttle.maxAngularVelocity
                 ]
        # Get the SCUTTLE's latest location
        x,y = scuttle.getGlobalPosition()

        # Print the location of the SCUTTLE
        print('Global Position: {}, {}'.format(round(x, 3), round(y, 3)))

        # Run loop at 50Hz
        time.sleep(1/50)

except KeyboardInterrupt:

    pass

finally:
    # Clean up.
    gamepad.close()
    scuttle.stop()
    print('Stopped.')