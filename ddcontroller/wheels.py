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

        """A class for representing and controlling a single wheel.

        This class represents a single wheel, including its motor and encoder, and provides methods for controlling the speed of the motor and calculating the rotation and linear velocity of the wheel.

        Attributes:
        closed_loop (bool): A flag indicating whether closed loop control should be used to control the motor.
        motor (Motor): An instance of the Motor class representing the motor driving the wheel.
        encoder (AS5048B): An instance of the AS5048B class representing the encoder attached to the wheel.
        pid (PID): An instance of the PID class representing the PID controller used for closed loop control.
        radius (float): The radius of the wheel, in meters.
        pulley_ratio (float): The ratio of the number of teeth on the motor pulley to the number of teeth on the wheel pulley.
        rpm (float): The rotational speed of the motor, in revolutions per minute.
        max_angular_velocity (float): The maximum angular velocity of the wheel, in radians per second, based on the maximum RPM of the motor.
        target_angular_velocity (float): The target angular velocity of the system, in radians per second, used to control the motor.
        position (int): The raw encoder position of the wheel, in ticks.
        timestamp (int): The timestamp of the last measurement of the encoder position, in nanoseconds.
        target_velocity (float): The target linear velocity of the wheel, in meters per second.
        angular_velocity (float): The current angular velocity of the wheel, in radians per second.
        linear_velocity (float): The current linear velocity of the wheel, in meters per second.
        rollover_limit (int): The limit at which the encoder position will rollover, in ticks.
        _positions (deque): A deque object containing the last two encoder positions of the wheel, in ticks.
        _timestamps (deque): A deque object containing the timestamps of the last two encoder position measurements, in nanoseconds.

        Args:
        motor_pins (tuple): A tuple containing the pins of the motor driver connected to the motor.
        pwm_frequency (int): The frequency of the pulse width modulation used to control the motor, in hertz.
        i2c_bus (int): The number of the I2C bus on which the encoder is connected.
        encoder_address (int): The I2C address of the encoder.
        wheel_radius (float): The radius of the wheel, in meters.
        motor_pulley_teeth (int): The number of teeth on the pulley attached to the motor shaft.
        wheel_pulley_teeth (int): The number of teeth on the pulley attached to the wheel hub.
        motor_decay_mode (str, optional): The decay mode of the motor. Can be 'FAST' or 'SLOW'. Defaults to 'FAST'.
        invert_motor (bool, optional): A flag indicating whether the motor should be inverted. Defaults to False.
        invert_encoder (bool, optional): A flag indicating whether the encoder readings should be inverted. Defaults to False.
        closed_loop (bool, optional): A flag indicating whether closed loop control should be used to control the motor. Defaults to False.
        Kp (int, optional): The proportional gain of the PID controller. Defaults to 0.
        Ki (int, optional): The integral gain of the PID controller. Defaults to 0.
        Kd (int, optional): The derivative gain of the PID controller. Defaults to 0.
        """


        self.closed_loop = closed_loop

        self.motor = motor.Motor(
            motor_pins,
            pwm_frequency,
            decay_mode=motor_decay_mode,
            invert=invert_motor,
        )

        self.encoder = AS5048B(
            encoder_address,
            bus=i2c_bus,
            invert=invert_encoder
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
        """Update the encoder position and timestamp.

        This method updates the position and timestamp of the encoder by reading the current position and adding it to a queue of positions and timestamps. The most recent position and timestamp are then stored as attributes of the Wheel object.
        """
        # Append new position to_positions queue.
        # This will push out the oldest item in the queue
        self._positions.append(self.encoder.read_position())
        # Append new timestamp to _timestamps queue.
        # This will push out the oldest item in the queue
        self._timestamps.append(time.monotonic_ns())
        self.position = self._positions[1]  # set latest position
        self.timestamp = self._timestamps[1]  # set timestamp of latest data

    def get_rotation(self):
        """Get the rotation of the wheel.

        This method calculates and returns the rotation of the wheel, in ticks, based on the positions of the encoder at two different points in time. It also accounts for rollover, where the encoder position resets after reaching a certain value. The rotation is calculated by taking the difference between the encoder positions and applying a pulley ratio to convert from motor pulley rotation to wheel rotation.

        Returns:
        int: The rotation of the wheel, in ticks.
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
        """Get the distance traveled by the wheel.

        This method calculates and returns the distance traveled by the wheel, in meters, based on the rotation of the wheel and the radius of the wheel.

        Returns:
        float: The distance traveled by the wheel.
        """

        # get wheel rotation between measurements
        rotation = self.get_rotation()
        distance = (2 * np.pi * self.radius) * (
            rotation / self.encoder.resolution
        )  # calculate distance traveled in wheel rotation
        return distance  # return distance traveled in meters

    def get_linear_velocity(self):  # get wheel linear velocity
        """Get the current linear velocity of the wheel.

        This method calculates and returns the current linear velocity of the wheel, in meters per second, based on the distance traveled by the wheel and the elapsed time between measurements.

        Returns:
        float: The current linear velocity of the wheel.
        """
        distance = self.get_travel()  # get wheel travel
        # calculate delta_time, convert from ns to s
        delta_time = (self._timestamps[1] - self._timestamps[0]) / 1e9
        # calculate wheel linear velocity
        self.linear_velocity = distance / delta_time
        return self.linear_velocity

    def get_angular_velocity(self):
        """Get the current angular velocity of the wheel.

        This method calculates and returns the current angular velocity of the wheel, in radians per second, based on the rotation of the wheel and the elapsed time between measurements.

        Returns:
        float: The current angular velocity of the wheel.
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
        """Set the target angular velocity of the wheel.

        This method sets the target angular velocity of the wheel, which is used to control the speed of the motor. The motor's duty cycle is adjusted based on the target angular velocity, using either open loop or closed loop control.

        Args:
        angular_velocity (float): The target angular velocity to set, in radians per second.
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
        """Stop the motor driving the wheel.

        This method stops the motor driving the wheel by setting the duty cycle to 0.
        """
        self.motor.stop()
