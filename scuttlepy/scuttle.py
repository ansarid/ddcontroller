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

import os
import yaml
import time
import threading
import numpy as np

# import RPi.GPIO as GPIO

from . import wheels
from .constants import *


class SCUTTLE:
    def __init__(self, config=None):
        """_summary_

        Args:
            config (_type_, optional): _description_. Defaults to None.
        """
        # GPIO.setmode(GPIO.BOARD)

        settings = settings(file=config)

        self.heading = 0
        self.velocity = 0
        self.angular_velocity = 0
        self.global_position = [0, 0]

        self.heading_offset = 0

        self.wheel_base = settings.wheel_base

        self.wheel_radius = settings.wheel_radius

        self.max_velocity = settings.maximum_linear_velocity
        self.max_angular_velocity = settings.maximum_angular_velocity

        self.left_wheel = wheels.Wheel(
            settings.left_wheel_motor_pins,
            settings.i2c_bus,
            settings.motor_pwm_frequency,
            settings.left_wheel_encoder_address,
            invert_encoder=settings.left_wheel_encoder_invert,
            invert_motor=settings.left_wheel_motor_invert,
        )

        self.right_wheel = wheels.Wheel(
            settings.right_wheel_motor_pins,
            settings.i2c_bus,
            settings.motor_pwm_frequency,
            settings.right_wheel_encoder_address,
            invert_encoder=settings.right_wheel_encoder_invert,
            invert_motor=settings.right_wheel_motor_invert,
        )

        self.wheel_speeds = [0, 0]
        self.target_motion = [0, 0]
        self._loop_start = time.monotonic_ns()

        self._time_initial = time.monotonic_ns()
        self._time_final = 0
        self._angular_displacement = 0
        self._forward_displacement = 0
        self._wheel_increments = np.array([0, 0])

        self.stopped = False

        self._loop_freq = 50  # target wheel loop frequency (hz)
        self._wait = (
            1 / self._loop_freq
        )  # corrected wait time between encoder measurements (s)

        self.wheels_thread = threading.thread(
            target=self._wheels_loop
        )  # create wheel loop thread object
        self.wheels_thread.start()  # start wheel loop thread object

    def sleep(self, start_time):
        """_summary_

        Args:
            start_time (_type_): _description_
        """
        # measure time since start and subtract from sleep time
        sleep_time = sorted(
            [self._wait - ((time.monotonic_ns() - start_time) / 1e9), 0]
        )[1]

        time.sleep(sleep_time)

    def _wheels_loop(self):
        """_summary_"""
        while not self.stopped:

            start_time = time.monotonic_ns()  # record loop start time

            self.left_wheel.update()  # update left wheel readings
            self.right_wheel.update()  # update right wheel readings

            (
                self.velocity,
                self.angular_velocity,
            ) = self.get_motion()  # get scuttle linear and angular velocities

            left_wheel_travel = self.left_wheel.get_travel()
            right_wheel_travel = self.right_wheel.get_travel()

            wheelbase_travel = (
                left_wheel_travel + right_wheel_travel
            ) / 2  # calculate wheel displacement

            self.global_position = [
                self.global_position[0]
                + (
                    wheelbase_travel * np.cos(self.heading)
                ),  # calculate global x position
                self.global_position[1]
                + (
                    wheelbase_travel * np.sin(self.heading)
                ),  # calculate global y position
            ]

            self.set_heading(
                # calculate and update global heading
                self.heading
                + ((right_wheel_travel - left_wheel_travel) / self.wheel_base)
            )

            self.sleep(start_time)

            # print loop time in ms
            # print((time.monotonic_ns()-start_time)/1e6)

        self.right_wheel.stop()
        self.left_wheel.stop()

    def stop(self):
        """_summary_"""
        self.set_motion([0, 0])
        self.stopped = True
        self.wheels_thread.join()

    def set_global_position(self, pos):
        """_summary_

        Args:
            pos (_type_): _description_

        Returns:
            _type_: _description_
        """
        self.global_position = pos
        return self.global_position

    def offset_heading(self, offset):
        """_summary_

        Args:
            offset (_type_): _description_

        Returns:
            _type_: _description_
        """
        self.heading_offset = offset
        heading = self.heading + self.heading_offset
        if heading < -np.pi:
            heading += 2 * np.pi
        elif heading > np.pi:
            heading -= 2 * np.pi
        self.set_headin(heading)
        return self.heading

    def set_heading(self, heading):
        """_summary_

        Args:
            heading (_type_): _description_

        Returns:
            _type_: _description_
        """
        self.heading = heading
        return self.heading

    def get_global_position(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        return self.global_position

    def get_heading(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        return self.heading

    def get_linear_velocity(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        return self.velocity

    def get_angular_velocity(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        return self.angular_velocity

    def set_linear_velocity(self, linear_velocity):
        """_summary_

        Args:
            linear_velocity (_type_): _description_

        Returns:
            _type_: _description_
        """
        self.target_motion[0] = linear_velocity
        self.set_motion(self.target_motion)
        return self.target_motion

    def set_angular_velocity(self, angular_velocity):
        """_summary_

        Args:
            angular_velocity (_type_): _description_

        Returns:
            _type_: _description_
        """
        self.target_motion[1] = angular_velocity
        self.set_motion(self.target_motion)
        return self.target_motion

    def set_motion(self, target_motion):
        """_summary_

        Args:
            target_motion (_type_): _description_
        """
        self.target_motion = target_motion

        A = np.array(
            [
                [1 / self.wheel_radius, -(self.wheel_base / 2) / self.wheel_radius],
                [1 / self.wheel_radius, (self.wheel_base / 2) / self.wheel_radius],
            ]
        )

        B = np.array([target_motion[0], target_motion[1]])

        C = np.matmul(A, B)

        self.left_wheel.set_angular_velocity(C[0])
        self.right_wheel.set_angular_velocity(C[1])

    def get_motion(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        A = np.array(
            [
                [self.wheel_radius / 2, self.wheel_radius / 2],
                [
                    -self.wheel_radius / (2 * (self.wheel_base / 2)),
                    (self.wheel_radius) / (2 * (self.wheel_base / 2)),
                ],
            ]
        )

        B = np.array(
            [
                self.left_wheel.get_angular_velocity(),
                self.right_wheel.get_angular_velocity(),
            ]
        )

        C = np.matmul(A, B)

        self.velocity = C[0]
        self.angular_velocity = C[1]

        return [self.velocity, self.angular_velocity]
