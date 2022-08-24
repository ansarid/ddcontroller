#!/usr/bin/python3

'''
This file is part of the robotPy library (https://github.com/ansarid/ddcontroller).
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

import numpy as np
from smbus2 import SMBus


class Encoder:
    """_summary_
        Encoder
    """
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


    def get_position(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        return self.position  # Return Raw encoder position (0 to 16384)

    def get_angle(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        return self.angle  # Return encoder angle in radians

    def get_magnitude(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        return self.magnitude  # Return encoder magnitude

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
        self.read_position()  # Read encoder position
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
