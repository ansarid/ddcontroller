#!/usr/bin/env python3

'''
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
'''

import time
import numpy as np
from ddcontroller.wheels import Wheel
from ruamel.yaml import YAML

yaml = YAML(typ='safe')
config = yaml.load(open('/opt/ddcontroller/config/default.yaml',"r").read())

# Create right wheel object
left_wheel = Wheel(
    digital_pin=config['robot']['left_wheel']['motor']['digital_pin'],
    pwm_pin=config['robot']['left_wheel']['motor']['pwm_pin'],
    pwm_frequency=config['robot']['left_wheel']['motor']['pwm_frequency'],
    i2c_bus=config['robot']['left_wheel']['encoder']['i2c_bus'],
    encoder_address=config['robot']['left_wheel']['encoder']['address'],
    wheel_radius=config['robot']['left_wheel']['wheel_radius'],
    motor_pulley_teeth=config['robot']['left_wheel']['motor_pulley_teeth'],
    wheel_pulley_teeth=config['robot']['left_wheel']['wheel_pulley_teeth'],
    invert_motor=config['robot']['left_wheel']['motor']['invert'],
    invert_encoder=config['robot']['left_wheel']['encoder']['invert'],
)

right_wheel = Wheel(
    digital_pin=config['robot']['right_wheel']['motor']['digital_pin'],
    pwm_pin=config['robot']['right_wheel']['motor']['pwm_pin'],
    pwm_frequency=config['robot']['right_wheel']['motor']['pwm_frequency'],
    i2c_bus=config['robot']['right_wheel']['encoder']['i2c_bus'],
    encoder_address=config['robot']['right_wheel']['encoder']['address'],
    wheel_radius=config['robot']['right_wheel']['wheel_radius'],
    motor_pulley_teeth=config['robot']['right_wheel']['motor_pulley_teeth'],
    wheel_pulley_teeth=config['robot']['right_wheel']['wheel_pulley_teeth'],
    invert_motor=config['robot']['right_wheel']['motor']['invert'],
    invert_encoder=config['robot']['right_wheel']['encoder']['invert'],
)

try:

    # Create infinite loop
    while True:

        # Update wheel measurements
        left_wheel.update()
        right_wheel.update()

        # Set wheel angular velocity to 2*pi
        left_wheel.motor.set_duty(1)
        right_wheel.motor.set_duty(1)

        print((2*(left_wheel.get_angular_velocity()*60)/(np.pi*2)), (2*(right_wheel.get_angular_velocity()*60)/(np.pi*2)))

        # Run loop at 50Hz
        time.sleep(1/50)

except KeyboardInterrupt:

    print('Stopping...')

finally:
    # Clean up.
    left_wheel.stop()
    right_wheel.stop()
    print('Stopped.')