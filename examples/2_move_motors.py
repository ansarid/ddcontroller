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
from ddcontroller.motor import Motor

# Create motor object (digital_pin, pwm_pin, pwm_frequency)
motor = Motor(11, 12, 150)

try:

    # Create infinite loop
    while True:

        # For loop iterating through values from 100 to -100 with an increment of -1
        for duty in range(100, -100, -1):

            # Divide duty by 100 because duty cycle input needs to be between -1 and 1
            duty /= 100

            # Print the current duty cycle
            print('Duty:', duty)

            # Set the duty cycle to the motor
            motor.set_duty(duty)

            # Run loop at 50Hz
            time.sleep(1/50)

except KeyboardInterrupt:
    print('Stopping...')

finally:
    # Clean up.
    motor.stop()
    print('Stopped.')