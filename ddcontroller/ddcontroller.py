#!/usr/bin/env python3


#  This file is part of the ddcontroller library (https://github.com/ansarid/ddcontroller).
#  Copyright (C) 2022  Daniyal Ansari

#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.

#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.


import time
import threading
import numpy as np
from simple_pid import PID
from ruamel.yaml import YAML

from . import wheels

yaml = YAML(typ='safe')


class DDRobot:

    def __init__(self, config_path='/opt/ddcontroller/config/default.yaml', debug=False):

        """Initialize a DDRobot object.

        This method initializes a DDRobot object by setting up the wheels and loading the configuration from a YAML file. The robot has various attributes such as its heading, linear velocity, and global position, as well as target motion, heading, and position. The robot also has various constants such as its wheel base, maximum linear and angular velocities, and tolerance for reaching the target position.

        Args:
        config_path (str, optional): The path to the YAML configuration file. Defaults to '/opt/ddcontroller/config/default.yaml'.
        debug (bool, optional): Whether to print debugging messages. Defaults to False.
        """

        self.config = yaml.load(open(config_path, "r", encoding="utf-8").read())
        self.debug = debug

        if self.debug:
            print(f"Loaded config from: {config_path}\nLabeled: {self.config['label']}")

        self.heading = 0
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.global_position = [0, 0]

        self.target_motion = [0, 0]
        self.target_heading = self.heading
        self.target_position = [0, 0]
        self.position_tolerance = 0.1
        self.reached_target_position = False
        self.heading_error = 0
        self.position_error = 0
        self.backwards = False

        self.odometry_frequency = 0
        self.heading_controller_frequency = 0
        self.position_controller_frequency = 0

        self.control_level = 1
        self.running = True

        self.wheel_base = self.config['robot']['wheel_base']

        self.max_linear_velocity = self.config['robot']['max_linear_velocity']
        self.max_angular_velocity = self.config['robot']['max_angular_velocity']

        self.max_traveling_linear_velocity = self.config['robot']['max_traveling_linear_velocity']
        self.max_traveling_angular_velocity = self.config['robot']['max_traveling_angular_velocity']

        self.left_wheel = wheels.Wheel(
            motor_pins=self.config['robot']['left_wheel']['motor']['pins'],
            pwm_frequency=self.config['robot']['left_wheel']['motor']['pwm_frequency'],
            motor_decay_mode=self.config['robot']['left_wheel']['motor']['decay_mode'],
            i2c_bus=self.config['robot']['left_wheel']['encoder']['i2c_bus'],
            encoder_address=self.config['robot']['left_wheel']['encoder']['address'],
            wheel_radius=self.config['robot']['left_wheel']['wheel_radius'],
            motor_pulley_teeth=self.config['robot']['left_wheel']['motor_pulley_teeth'],
            wheel_pulley_teeth=self.config['robot']['left_wheel']['wheel_pulley_teeth'],
            invert_motor=self.config['robot']['left_wheel']['motor']['invert'],
            invert_encoder=self.config['robot']['left_wheel']['encoder']['invert'],
            closed_loop=self.config['robot']['left_wheel']['closed_loop'],
            Kp=self.config['robot']['left_wheel']['Kp'],
            Ki=self.config['robot']['left_wheel']['Ki'],
            Kd=self.config['robot']['left_wheel']['Kd'],
        )

        self.right_wheel = wheels.Wheel(
            motor_pins=self.config['robot']['right_wheel']['motor']['pins'],
            pwm_frequency=self.config['robot']['right_wheel']['motor']['pwm_frequency'],
            motor_decay_mode=self.config['robot']['right_wheel']['motor']['decay_mode'],
            i2c_bus=self.config['robot']['right_wheel']['encoder']['i2c_bus'],
            encoder_address=self.config['robot']['right_wheel']['encoder']['address'],
            wheel_radius=self.config['robot']['right_wheel']['wheel_radius'],
            motor_pulley_teeth=self.config['robot']['right_wheel']['motor_pulley_teeth'],
            wheel_pulley_teeth=self.config['robot']['right_wheel']['wheel_pulley_teeth'],
            invert_motor=self.config['robot']['right_wheel']['motor']['invert'],
            invert_encoder=self.config['robot']['right_wheel']['encoder']['invert'],
            closed_loop=self.config['robot']['right_wheel']['closed_loop'],
            Kp=self.config['robot']['right_wheel']['Kp'],
            Ki=self.config['robot']['right_wheel']['Ki'],
            Kd=self.config['robot']['right_wheel']['Kd'],
        )

        self.heading_pid = PID(self.config['robot']['heading_Kp'],
                               self.config['robot']['heading_Ki'],
                               self.config['robot']['heading_Kd'],
                               setpoint=0)

        self.heading_pid.output_limits = (-self.max_angular_velocity, self.max_angular_velocity)

        self.loop_freq = self.config['robot']['wheel_frequency']  # target wheel loop frequency (hz)

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

            self.heading = np.arctan2(
                                        np.sin(self.heading + ((right_wheel_travel - left_wheel_travel) / self.wheel_base)),
                                        np.cos(self.heading + ((right_wheel_travel - left_wheel_travel) / self.wheel_base))
                                    )

            self.sleep(start_time)
            self.odometry_frequency = 1000/((time.monotonic_ns()-start_time)/1e6)

    def _heading_controller(self):

        while self.running:

            start_time = time.monotonic_ns()  # record loop start time

            if self.control_level >= 2:

                heading_error = self.get_heading() - self.target_heading

                if self.backwards:
                    heading_error+=np.pi
                else:
                    pass

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
                    self.set_heading(target_heading, max_angular_velocity=self.max_angular_velocity)

                    if not self.backwards:
                        if self.max_traveling_linear_velocity:
                            self.set_linear_velocity(self.max_traveling_linear_velocity*abs(1-(self.target_motion[1]/self.max_angular_velocity)))
                        else:
                            self.set_linear_velocity(self.max_linear_velocity*abs(1-(self.target_motion[1]/self.max_angular_velocity)))
                    else:
                        if self.max_traveling_linear_velocity:
                            self.set_linear_velocity(-(self.max_traveling_linear_velocity*abs(1-(self.target_motion[1]/self.max_angular_velocity))))
                        else:
                            self.set_linear_velocity(-(self.max_linear_velocity*abs(1-(self.target_motion[1]/self.max_angular_velocity))))

                    self.sleep(start_time)
                    self.position_controller_frequency = 1000/((time.monotonic_ns()-start_time)/1e6)

                self.reached_target_position = True

            self.sleep(start_time)
            self.position_controller_frequency = 1000/((time.monotonic_ns()-start_time)/1e6)

    def stop(self):

        """Stop the robot.

        This method stops the robot by setting the motion to 0, stopping the control threads, and stopping the wheels.
        """
        self.set_motion([0, 0])
        self.running = False
        self.odometry_thread.join()
        self.heading_controller_thread.join()
        self.position_controller_thread.join()
        self.right_wheel.stop()
        self.left_wheel.stop()

    def define_heading(self, heading):
        """Define the heading of the robot.

        This method defines the heading of the robot by taking the arctangent of the sine and cosine of the input heading, constraining the heading to be between -pi and pi. The defined heading is then stored as an attribute of the DDRobot object and returned.

        Args:
        heading (float): The heading of the robot in radians.

        Returns:
        float: The defined heading of the robot in radians, constrained to be between -pi and pi.
        """

        self.heading = np.arctan2(np.sin(heading), np.cos(heading))
        return self.heading

    def set_heading(self, target_heading, max_angular_velocity=None):
        """Set the heading of the robot.

        This method sets the target heading of the robot by taking the arctangent of the sine and cosine of the input heading, constraining the heading to be between -pi and pi. The maximum angular velocity of the robot can also be set as an optional parameter. The control level of the robot is set to 2, indicating that heading control is active.

        Args:
        target_heading (float): The target heading of the robot in radians.
        max_angular_velocity (float, optional): The maximum angular velocity of the robot in radians per second. Defaults to None.

        Returns:
        None
        """

        if max_angular_velocity:
            self.heading_pid.output_limits = (-max_angular_velocity, max_angular_velocity)

        self.control_level = 2
        self.target_heading = np.arctan2(np.sin(target_heading), np.cos(target_heading))

    def get_heading(self):
        """Get the heading of the robot.

        This method returns the heading of the robot as an attribute of the DDRobot object.

        Returns:
        float: The heading of the robot in radians.
        """
        return self.heading

    def define_global_position(self, position):
        """Define the global position of the robot.

        This method sets the global position of the robot as an attribute of the DDRobot object.

        Args:
        position (list): A list containing the x and y position of the robot.

        Returns:
        list: The global position of the robot.
        """
        self.global_position = position
        return self.global_position

    def get_global_position(self):
        """Get the global position of the robot.

        This method returns the global position of the robot as an attribute of the DDRobot object.

        Returns:
        list: The global position of the robot, as a list containing the x and y position.
        """
        return self.global_position

    def get_linear_velocity(self):
        """Get the linear velocity of the robot.

        This method returns the linear velocity of the robot as an attribute of the DDRobot object.

        Returns:
        float: The linear velocity of the robot, in meters per second.
        """
        return self.linear_velocity

    def get_angular_velocity(self):
        """Get the angular velocity of the robot.

        This method returns the angular velocity of the robot as an attribute of the DDRobot object.

        Returns:
        float: The angular velocity of the robot, in radians per second.
        """
        return self.angular_velocity

    def set_linear_velocity(self, linear_velocity):

        """Set the desired linear velocity of the robot.

        Args:
            linear_velocity (float): The linear velocity to set in meters per second.

        Returns:
            list: The updated target motion of the robot.
        """

        self.target_motion[0] = linear_velocity
        self.set_motion(self.target_motion)
        return self.target_motion

    def set_angular_velocity(self, angular_velocity):
        """Set the target angular velocity for the robot.

        Args:
            angular_velocity (float): The desired angular velocity of the robot in radians per second.

        Returns:
            list: The current target motion of the robot, where the first element is the linear velocity and the second element is the angular velocity.
        """
        self.target_motion[1] = angular_velocity
        self.set_motion(self.target_motion)
        return self.target_motion

    def set_motion(self, target_motion):
        """
        Set the target linear and angular velocities for the robot.

        Args:
            target_motion (list): A list containing the target linear velocity (m/s) as the first element
                                 and the target angular velocity (rad/s) as the second element.

        Returns:
            numpy.ndarray: An array containing the target angular velocities (rad/s) for the left and right wheels.
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
            print(f"Left wheel requested angular velocity exceeded maximum({self.left_wheel.max_angular_velocity}): {C[0]}")
        if C[1] > self.right_wheel.max_angular_velocity and self.debug:
            print(f"Right wheel requested angular velocity exceeded maximum({self.right_wheel.max_angular_velocity}): {C[1]}")

        self.left_wheel.set_angular_velocity(C[0])
        self.right_wheel.set_angular_velocity(C[1])

        return C

    def get_motion(self):
        """
        Calculate and return the current linear and angular velocities of the robot.

        Returns:
            list: A list of two floats, representing the linear and angular velocities of the robot, in that order.
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

    def go_to(self, target_position, tolerance=0.1, max_linear_velocity=None, max_angular_velocity=None, backwards=False):
        """
        Moves the robot to the specified target position using a position controller.
        The robot will continue to move towards the target position until it is within the specified tolerance.

        Args:
            target_position (list): The target position as a 2-element list [x, y] in global coordinates.
            tolerance (float, optional): The tolerance for reaching the target position in meters. Defaults to 0.1.
            max_linear_velocity (float, optional): The maximum linear velocity the robot will use to reach the target position in m/s. If not provided, the default value specified in the configuration file will be used.
            max_angular_velocity (float, optional): The maximum angular velocity the robot will use to reach the target position in rad/s. If not provided, the default value specified in the configuration file will be used.
            backwards (bool, optional): Whether the robot should move backwards to reach the target position. Defaults to False.
        """
        self.target_position = target_position
        self.position_tolerance = tolerance
        self.backwards=backwards

        if max_linear_velocity:
            self.max_linear_velocity = max_linear_velocity
        if max_angular_velocity:
            self.max_angular_velocity = max_angular_velocity

        self.reached_target_position = False
        self.control_level = 2
