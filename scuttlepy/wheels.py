#!/usr/bin/python3

import time
import numpy as np
from collections import deque

from . import motor
from . import encoder
from .constants import *

# import RPi.GPIO as GPIO

# if GPIO.getmode() is None:
#     GPIO.setmode(GPIO.BOARD)


class Wheel:
    def __init__(
        self,
        motor_output,
        bus,
        motor_pwm_freq,
        encoder_address,
        wheel_radius=0.04165,
        motor_pulley_teeth=13,
        wheel_pulley_teeth=25,
        invert_motor=False,
        invert_encoder=False,
    ):
        """_summary_

        Args:
            motor_output (_type_): _description_
            bus (_type_): _description_
            motor_pwm_freq (_type_): _description_
            encoder_address (_type_): _description_
            wheel_radius (float, optional): _description_. Defaults to 0.04165.
            motor_pulley_teeth (int, optional): _description_. Defaults to 13.
            wheel_pulley_teeth (int, optional): _description_. Defaults to 25.
            invert_motor (bool, optional): _description_. Defaults to False.
            invert_encoder (bool, optional): _description_. Defaults to False.
        """
        self.invert_motor = invert_motor
        self.invert_encoder = invert_encoder

        self.motor = motor.motor(
            motor_output,
            frequency=motor_pwm_freq,
            invert=invert_motor,
        )

        self.encoder = encoder.encoder(
            encoder_address, bus=bus, invert=self.invert_encoder
        )

        self.radius = wheel_radius
        self.pulley_ratio = motor_pulley_teeth / wheel_pulley_teeth

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

    def get_rotation(self):  # calculate the increment of a wheel in ticks
        """_summary_

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

        if self.target_angular_velocity > 0.15:
            duty = (0.098 * self.target_angular_velocity) + 0.148
        elif self.target_angular_velocity < -0.15:
            duty = (0.098 * self.target_angular_velocity) - 0.148
        else:
            duty = 0

        # set duty cycle to motor
        self.motor.set_duty(duty)

    def stop(self):
        """_summary_"""
        self.motor.stop()
