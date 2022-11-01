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
from ruamel.yaml import YAML
from ddcontroller.wheels import Wheel

yaml = YAML(typ='safe')
config = yaml.load(open('/opt/ddcontroller/config/default.yaml',"r").read())

def radians2rpm(radians):
    return (2*(radians*60)/(np.pi*2))

left_wheel = Wheel(
    motor_pins=config['robot']['left_wheel']['motor']['pins'],
    pwm_frequency=config['robot']['left_wheel']['motor']['pwm_frequency'],
    motor_decay_mode=config['robot']['left_wheel']['motor']['decay_mode'],
    i2c_bus=config['robot']['left_wheel']['encoder']['i2c_bus'],
    encoder_address=config['robot']['left_wheel']['encoder']['address'],
    wheel_radius=config['robot']['left_wheel']['wheel_radius'],
    motor_pulley_teeth=config['robot']['left_wheel']['motor_pulley_teeth'],
    wheel_pulley_teeth=config['robot']['left_wheel']['wheel_pulley_teeth'],
    invert_motor=config['robot']['left_wheel']['motor']['invert'],
    invert_encoder=config['robot']['left_wheel']['encoder']['invert'],
)

right_wheel = Wheel(
    motor_pins=config['robot']['right_wheel']['motor']['pins'],
    pwm_frequency=config['robot']['right_wheel']['motor']['pwm_frequency'],
    motor_decay_mode=config['robot']['right_wheel']['motor']['decay_mode'],
    i2c_bus=config['robot']['right_wheel']['encoder']['i2c_bus'],
    encoder_address=config['robot']['right_wheel']['encoder']['address'],
    wheel_radius=config['robot']['right_wheel']['wheel_radius'],
    motor_pulley_teeth=config['robot']['right_wheel']['motor_pulley_teeth'],
    wheel_pulley_teeth=config['robot']['right_wheel']['wheel_pulley_teeth'],
    invert_motor=config['robot']['right_wheel']['motor']['invert'],
    invert_encoder=config['robot']['right_wheel']['encoder']['invert'],
)

try:

    # Set motors to maximum duty cycle
    left_wheel.motor.set_duty(1)
    right_wheel.motor.set_duty(1)

    # Create infinite loop
    while True:

        # Update wheel measurements
        left_wheel.update()
        right_wheel.update()

        print(f"Left Wheel Speed: {round(left_wheel.get_angular_velocity(),3)} rad/s\t{round(radians2rpm(left_wheel.get_angular_velocity()),1)} rpm\tRight Wheel Speed: {round(right_wheel.get_angular_velocity(),3)} rad/s\t{round(radians2rpm(right_wheel.get_angular_velocity()),1)} rpm")

        # Run loop at 50Hz
        time.sleep(1/50)

except KeyboardInterrupt:

    print('Stopping...')

finally:
    # Clean up.
    left_wheel.stop()
    right_wheel.stop()
    print('Stopped.')