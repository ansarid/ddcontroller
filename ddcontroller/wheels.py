import time
import numpy as np
from collections import deque

from . import motor
from as5600l import AS5600L

class Wheel:

    def __init__(
        self,
        motor_pins,
        wheel_radius,
        motor_decay_mode='SLOW',
        i2c_bus=1,
        encoder_address=0x41,
        invert_motor=False,
        invert_encoder=False,
        ):

        self.motor = motor.Motor(
            motor_pins,
            rpm=255,
            invert=invert_motor,
            decay=motor_decay_mode
        )

        self.encoder = AS5600L(
            encoder_address,
            bus=i2c_bus,
            invert=invert_encoder
        )

        self.radius = wheel_radius

        self.rpm = self.motor.rpm
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
        self.rollover_limit = self.encoder.resolution/2

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

        if -rotation > self.rollover_limit:
            rotation += self.encoder.resolution
        elif rotation > self.rollover_limit:
            rotation -= self.encoder.resolution

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

        # duty = ((-(self.motor.max_duty)*2)/(-self.max_angular_velocity*2))*self.target_angular_velocity
        duty = angular_velocity/(self.motor.rpm * (2 * np.pi / 60))
        # print(self.motor.rpm,angular_velocity, duty)
        # set duty cycle to motor
        self.motor.set_duty(duty)

    def stop(self):
        """Stop the motor driving the wheel.

        This method stops the motor driving the wheel by setting the duty cycle to 0.
        """
        self.motor.stop()
