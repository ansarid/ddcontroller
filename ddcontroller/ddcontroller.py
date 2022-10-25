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
import threading
import numpy as np

from . import wheels
from simple_pid import PID
from ruamel.yaml import YAML

yaml = YAML(typ='safe')

class DDRobot:

    """_summary_
    DDRobot
    """

    def __init__(self, config_path='/opt/ddcontroller/config/default.yaml', debug=False):
        """_summary_

        Args:
            config (_type_, optional): _description_. Defaults to None.
        """

        config = yaml.load(open(config_path,"r").read())

        self.debug = debug

        self.heading = 0
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.global_position = [0, 0]

        self.target_motion = [0, 0]
        self.target_heading = self.heading
        self.target_position = [0,0]
        self.reached_target_position = False
        self.position_tolerance = 0.1
        self.heading_error = 0
        self.position_error = 0

        # These'll need to be revisited. Did rough PID tuning.
        heading_Kp = 0.35
        heading_Ki = 0
        heading_Kd = 0

        self.odometry_frequency = 0
        self.heading_controller_frequency = 0
        self.position_controller_frequency = 0

        self.control_level = 1
        self.running = True

        self.wheel_base = config['robot']['wheel_base']

        self.max_linear_velocity = config['robot']['max_linear_velocity']
        self.max_angular_velocity = config['robot']['max_angular_velocity']

        self.max_traveling_linear_velocity = config['robot']['max_traveling_linear_velocity']
        self.max_traveling_angular_velocity = config['robot']['max_traveling_angular_velocity']

        self.left_wheel = wheels.Wheel(
            motor_pins=config['robot']['l_wheel']['motor']['pins'],
            pwm_frequency=config['robot']['l_wheel']['motor']['pwm_frequency'],
            i2c_bus=config['robot']['l_wheel']['encoder']['i2c_bus'],
            encoder_address=config['robot']['l_wheel']['encoder']['address'],
            wheel_radius=config['robot']['l_wheel']['wheel_radius'],
            motor_pulley_teeth=config['robot']['l_wheel']['motor_pulley_teeth'],
            wheel_pulley_teeth=config['robot']['l_wheel']['wheel_pulley_teeth'],
            invert_motor=config['robot']['l_wheel']['motor']['invert'],
            invert_encoder=config['robot']['l_wheel']['encoder']['invert'],
        )

        self.right_wheel = wheels.Wheel(
            motor_pins=config['robot']['r_wheel']['motor']['pins'],
            pwm_frequency=config['robot']['r_wheel']['motor']['pwm_frequency'],
            i2c_bus=config['robot']['r_wheel']['encoder']['i2c_bus'],
            encoder_address=config['robot']['r_wheel']['encoder']['address'],
            wheel_radius=config['robot']['r_wheel']['wheel_radius'],
            motor_pulley_teeth=config['robot']['r_wheel']['motor_pulley_teeth'],
            wheel_pulley_teeth=config['robot']['r_wheel']['wheel_pulley_teeth'],
            invert_motor=config['robot']['r_wheel']['motor']['invert'],
            invert_encoder=config['robot']['r_wheel']['encoder']['invert'],
        )

        self.heading_pid = PID(heading_Kp, heading_Ki, heading_Kd, setpoint=0)
        self.heading_pid.output_limits = (-self.max_angular_velocity, self.max_angular_velocity)
        self.heading_pid.setpoint = 0

        self.loop_freq = config['robot']['wheel_frequency']  # target wheel loop frequency (hz)

        self._wait = (
            1 / self.loop_freq
        )  # corrected wait time between encoder measurements (s)

        self.odometry_thread = threading.Thread(
            target=self._odometry_loop
        )  # create odometry thread object

        self.heading_controller_thread = threading.Thread(
            target=self._heading_controller
        )  # create heading controller thread object

        self.position_controller_thread = threading.Thread(
            target=self._position_controller
        )  # create position controller thread object

        self.odometry_thread.start()                # start odometry thread
        self.heading_controller_thread.start()      # start heading controller thread # Ideally we don't start this until it's needed
        self.position_controller_thread.start()     # start position contoller thread # Ideally we don't start this until it's needed

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

        return sleep_time

    def _odometry_loop(self):

        while self.running:

            start_time = time.monotonic_ns()  # record loop start time

            self.left_wheel.update()  # update left wheel readings
            self.right_wheel.update()  # update right wheel readings

            self.linear_velocity, self.angular_velocity  = self.get_motion()  # get robot linear and angular velocities

            left_wheel_travel = self.left_wheel.get_travel()
            right_wheel_travel = self.right_wheel.get_travel()

            wheelbase_travel = (
                left_wheel_travel + right_wheel_travel
            ) / 2  # calculate wheelbase displacement

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

            self._write_heading(
                # calculate and update global heading
                self.heading
                + ((right_wheel_travel - left_wheel_travel) / self.wheel_base)
            )

            self.sleep(start_time)
            self.odometry_frequency = 1000/((time.monotonic_ns()-start_time)/1e6)

    def _heading_controller(self):

        while self.running:

            start_time = time.monotonic_ns()  # record loop start time

            if self.control_level >= 2:

                heading_error = self.target_heading-self.get_heading()

                self.heading_error = heading_error
                self.heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

                angular_velocity = self.heading_pid(self.heading_error)
                self.set_angular_velocity(angular_velocity)

            self.sleep(start_time)
            self.heading_controller_frequency = 1000/((time.monotonic_ns()-start_time)/1e6)

    def _position_controller(self):

        def position_error():
            self.position_error = np.linalg.norm(np.array(self.target_position)-self.global_position)
            return self.position_error

        while self.running:

            start_time = time.monotonic_ns()  # record loop start time

            if self.control_level >= 2:

                self.reached_target_position = False

                while position_error() > self.position_tolerance and self.running:
                    start_time = time.monotonic_ns()  # record loop start time

                    target_heading = np.arctan2((self.target_position[1]-self.global_position[1]),(self.target_position[0]-self.global_position[0]))
                    self.set_heading(target_heading, max_angular_velocity=self.max_traveling_linear_velocity)

                    if self.max_traveling_linear_velocity:
                        self.set_linear_velocity(self.max_traveling_linear_velocity)
                    else:
                        self.set_linear_velocity(self.max_linear_velocity)

                    self.sleep(start_time)
                    self.position_controller_frequency = 1000/((time.monotonic_ns()-start_time)/1e6)

                self.reached_target_position = True

            self.sleep(start_time)
            self.position_controller_frequency = 1000/((time.monotonic_ns()-start_time)/1e6)

    def stop(self):
        """_summary_"""
        self.set_motion([0, 0])
        self.running = False
        self.odometry_thread.join()
        self.heading_controller_thread.join()
        self.position_controller_thread.join()
        self.right_wheel.stop()
        self.left_wheel.stop()

    def _write_heading(self, heading):
        """_summary_

        Args:
            heading (_type_): _description_

        Returns:
            _type_: _description_
        """

        self.heading = np.arctan2(np.sin(heading), np.cos(heading))
        return self.heading

    def _set_heading(self, target_heading):
        """_summary_

        Args:
            target_heading (_type_): _description_

        Returns:
            _type_: _description_
        """

        self.target_heading = np.arctan2(np.sin(target_heading), np.cos(target_heading))
        return self.target_heading

    def set_heading(self, target_heading, max_angular_velocity=None):
        """_summary_

        Args:
            target_heading (_type_): _description_

        Returns:
            _type_: _description_
        """

        if max_angular_velocity:
            self.heading_pid.output_limits = (-max_angular_velocity, max_angular_velocity)

        self.control_level = 2
        self._set_heading(target_heading)

    def get_heading(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        return self.heading

    def set_global_position(self, pos):
        """_summary_

        Args:
            pos (_type_): _description_

        Returns:
            _type_: _description_
        """
        self.global_position = pos
        return self.global_position

    def get_global_position(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        return self.global_position

    def get_linear_velocity(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        return self.linear_velocity

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

        A = np.array([
                      [ 1/self.left_wheel.radius, -(self.wheel_base/2)/self.left_wheel.radius],
                      [ 1/self.right_wheel.radius, (self.wheel_base/2)/self.right_wheel.radius]
                    ])

        B = np.array([target_motion[0],
                      target_motion[1]])

        C = np.matmul(A, B)

        if C[0] > self.left_wheel.max_angular_velocity and self.debug:
            print('Left wheel requested angular velocity exceeded maximum(',self.right_wheel.max_angular_velocity,'):', C[0])
        if C[1] > self.right_wheel.max_angular_velocity and self.debug:
            print('Right wheel requested angular velocity exceeded maximum(',self.right_wheel.max_angular_velocity,'):', C[1])

        self.left_wheel.set_angular_velocity(C[0])
        self.right_wheel.set_angular_velocity(C[1])

        return C

    def get_motion(self):
        """_summary_

        Returns:
            _type_: _description_
        """

        A = np.array(
            [
                [self.left_wheel.radius / 2, self.right_wheel.radius / 2],
                [
                    -self.left_wheel.radius / self.wheel_base,
                    self.right_wheel.radius / self.wheel_base,
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

        self.linear_velocity = C[0]
        self.angular_velocity = C[1]

        return [self.linear_velocity, self.angular_velocity]

    def go_to(self, target_position, tolerance=0.1, max_linear_velocity=None, max_angular_velocity=None):
        """_summary_

        Args:
            target_position (_type_): _description_
            tolerance (float, optional): _description_. Defaults to 0.1.
            max_linear_velocity (_type_, optional): _description_. Defaults to None.
            max_angular_velocity (_type_, optional): _description_. Defaults to None.
        """
        self.target_position = target_position
        self.position_tolerance = tolerance
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.control_level = 2
