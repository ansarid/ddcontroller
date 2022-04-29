#!/usr/bin/python3

'''
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
'''

import time
from .utils.gamepad import Gamepad
from scuttlepy import SCUTTLE

gamepad = Gamepad()
scuttle = SCUTTLE()

try:

    while True:

        # Get joystick x and y values and scale values to be between -1 and 1
        joystickX, joystickY = [((-2/255)*gamepad.axes['LEFT_X'])+1,
                                ((-2/255)*gamepad.axes['LEFT_Y'])+1
                                ]

        # Mutiply joystick x & y axis by scuttle max linear & angular
        # velocity to get linear & angular velocity
        motion = [
                  joystickY*scuttle.max_velocity,
                  joystickX*scuttle.max_angular_velocity
                  ]

        x, y = scuttle.get_global_position()

        print(
              'Global Position:',
              round(x, 3),
              ',',
              round(y, 3)
              )

        # Set motion to SCUTTLE
        scuttle.set_motion(motion)

        time.sleep(0.05)

except KeyboardInterrupt:

    pass

finally:

    gamepad.close()
    scuttle.stop()
