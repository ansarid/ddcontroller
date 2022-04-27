#!/usr/bin/python3

import numpy as np
from smbus2 import SMBus


class Encoder:
    def __init__(self, address, bus=1, invert=False):
        """_summary_

        Args:
            address (_type_): _description_
            bus (int, optional): _description_. Defaults to 1.
            invert (bool, optional): _description_. Defaults to False.
        """
        self.bus = SMBus(bus)  # I2C Bus
        self.address = address  # Encoder I2C address
        self.invert = invert  # Invert encoder direction
        self.resolution = 2 ** 14  # Define "ticks", 14 bit encoder
        self.position = self.read_position()  # Read position of encoder
        self.angle = self.read_angle()
        self.magnitude = self.read_magnitude()

    def read_position(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        pos = self.bus.read_i2c_block_data(
            self.address, 0xFE, 2
        )  # Request data from registers 0xFE & 0xFF of the encoder
        # Takes ~700 microseconds.
        self.position = (pos[0] << 6) | pos[
            1
        ]  # Remove unused bits 6 & 7 from byte 0xFF creating 14 bit value
        if self.invert:
            self.position = (
                self.resolution - self.position
            )  # If encoder is inverted, invert position
        return self.position  # Return Raw encoder position (0 to 16384)

    def read_angle(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        self.readPosition()  # Read encoder position
        self.angle = self.position * (
            (2 * np.pi) / self.resolution
        )  # Scale values to get radians
        return self.angle  # Return encoder angle in radians

    def read_magnitude(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        magnitude = self.bus.read_i2c_block_data(
            self.address, 0xFC, 2
        )  # Request data from registers 0xFC & 0xFD of the encoder
        # Takes ~700 microseconds.
        self.magnitude = (magnitude[0] << 6) | magnitude[
            1
        ]  # Remove unused bits 6 & 7 from byte 0xFD creating 14 bit value
        return self.magnitude  # Return encoder magnitude
