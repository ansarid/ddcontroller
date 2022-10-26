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
from collections import deque

from . import motor
from as5048b import AS5048B
from simple_pid import PID

class Wheel:

    """_summary_

    """

    def __init__(
        self,
        motor_pins,
        pwm_frequency,
        i2c_bus,
        encoder_address,
        wheel_radius,
        motor_pulley_teeth,
        wheel_pulley_teeth,
        motor_decay_mode='FAST',
        invert_motor=False,
        invert_encoder=False,
        closed_loop=False,
        Kp=0,
        Ki=0,
        Kd=0,
    ):

        """_summary_

        Args:
            motor_pins (_type_): _description_
            pwm_frequency (_type_): _description_
            motor_decay_mode (_type_): _description_
            i2c_bus (_type_): _description_
            encoder_address (_type_): _description_
            wheel_radius (_type_): _description_
            motor_pulley_teeth (_type_): _description_
            wheel_pulley_teeth (_type_): _description_
            invert_motor (bool, optional): _description_. Defaults to False.
            invert_encoder (bool, optional): _description_. Defaults to False.
            closed_loop (bool, optional): _description_. Defaults to False.
            Kp (int, optional): _description_. Defaults to 0.
            Ki (int, optional): _description_. Defaults to 0.
            Kd (int, optional): _description_. Defaults to 0.
        """

        self.invert_motor = invert_motor
        self.invert_encoder = invert_encoder
        self.closed_loop = closed_loop

        self.motor = motor.Motor(
            motor_pins,
            pwm_frequency,
            decay_mode=motor_decay_mode,
            invert=invert_motor,
        )

        self.encoder = AS5048B(
            encoder_address, bus=i2c_bus, invert=self.invert_encoder
        )

        if self.closed_loop:
            self.pid = PID(Kp, Ki, Kd, setpoint=0)
            self.pid.output_limits = (-self.motor.max_duty, self.motor.max_duty)

        self.radius = wheel_radius
        self.pulley_ratio = motor_pulley_teeth / wheel_pulley_teeth

        self.rpm = self.motor.rpm * self.pulley_ratio
        self.max_angular_velocity = (self.rpm/60)*(np.pi*2)

        # create target angular velocity
        self.target_angular_velocity = 0

        # stores raw encoder 'ticks'
        self.position = self.encoder.read_position()

        # stores age of data
        self.timestamp = time.monotonic_ns()
        self.target_velocity = 0
        self.angular_velocity = 0
        self.linear_velocity = 0

        # limit for rollover
        self.rollover_limit = self.pulley_ratio * self.encoder.resolution

        # create fifo queue object for wheel positions
        self._positions = deque([self.position, self.encoder.read_position()], maxlen=2)

        # create fifo queue object for wheel positions timestamps
        self._timestamps = deque([time.monotonic_ns()] * 2, maxlen=2)

        self.update()  # update values in object

    def update(self):
        """_summary_"""
        # Append new position to_positions queue.
        # This will push out the oldest item in the queue
        self._positions.append(self.encoder.read_position())
        # Append new timestamp to _timestamps queue.
        # This will push out the oldest item in the queue
        self._timestamps.append(time.monotonic_ns())
        self.position = self._positions[1]  # set latest position
        self.timestamp = self._timestamps[1]  # set timestamp of latest data

    def get_rotation(self):
        """_summary_
            Calculate the increment of a wheel in ticks.
        Returns:
            _type_: _description_
        """
        rotation = (
            self._positions[1] - self._positions[0]
        )  # calculate how much wheel has rotated
        # if the movement has rollover
        if -rotation >= self.rollover_limit:
            # handle forward rollover
            rotation = rotation + self.encoder.resolution
        # if movement has rollover in the negative direction
        if rotation >= self.rollover_limit:
            # handle reverse rollover
            rotation = rotation - self.encoder.resolution
        # go from motor pulley to wheel pulley
        rotation *= self.pulley_ratio
        # return wheel advancement in ticks
        return rotation

    def get_travel(self):  # calculate travel of the wheel in meters
        """_summary_

        Returns:
            _type_: _description_
        """
        # get wheel rotation between measurements
        rotation = self.get_rotation()
        distance = (2 * np.pi * self.radius) * (
            rotation / self.encoder.resolution
        )  # calculate distance traveled in wheel rotation
        return distance  # return distance traveled in meters

    def get_linear_velocity(self):  # get wheel linear velocity
        """_summary_

        Returns:
            _type_: _description_
        """
        distance = self.get_travel()  # get wheel travel
        # calculate delta_time, convert from ns to s
        delta_time = (self._timestamps[1] - self._timestamps[0]) / 1e9
        # calculate wheel linear velocity
        self.linear_velocity = distance / delta_time
        return self.linear_velocity

    def get_angular_velocity(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        # get wheel rotation between measurements
        rotation = self.get_rotation()
        # calculate delta_time, convert from ns to s
        delta_time = (self._timestamps[1] - self._timestamps[0]) / 1e9
        # speed produced from true wheel rotation (rad)
        self.angular_velocity = (
            rotation * (np.pi / (self.encoder.resolution / 2))
        ) / delta_time
        return self.angular_velocity

    def set_angular_velocity(self, angular_velocity):
        """_summary_

        Args:
            angular_velocity (_type_): _description_
        """
        self.target_angular_velocity = angular_velocity

        if not self.closed_loop:
            # Open loop control
            duty = ((-(self.motor.max_duty)*2)/(-self.max_angular_velocity*2))*self.target_angular_velocity

        elif self.closed_loop:
            # Closed loop PID control
            self.pid.setpoint = self.target_angular_velocity
            duty = self.pid(self.get_angular_velocity())

        # set duty cycle to motor
        self.motor.set_duty(duty)

    def stop(self):
        """_summary_"""
        self.motor.stop()
