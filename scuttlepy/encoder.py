#!/usr/bin/python3

# Import external libraries
from smbus2 import SMBus

class Encoder:

    def __init__(self, address, bus=1, invert=False):

        self.bus = SMBus(bus)
        self.address = address
        self.invert = invert
        self.resolution = 2**14                                             # Define "ticks", 14 bit encoder
        self.position = self.readPos()                                      # Read position of encoder
        self.angle = self.readAngle()
        self.magnitude = self.readMagnitude()

    def readPos(self):

        pos = self.bus.read_i2c_block_data(self.address, 0xFE, 2)           # Request data from registers 0xFE & 0xFF of the encoder
                                                                            # Takes ~700 microseconds.
        if not self.invert:
            self.position = (pos[0] << 6) | pos[1]                          # Remove unused bits 6 & 7 from byte 0xFF creating 14 bit value
        else:
            self.position = (2**14)-((pos[0] << 6) | pos[1])                # Remove unused bits 6 & 7 from byte 0xFF creating 14 bit value & invert the value

        return self.position                                                # Return Raw encoder position (0 to 16384)

    def readAngle(self):

        self.readPos()                                                      # Read encoder position

        self.angle = self.position * (360 / 2**14)                          # Scale values to get degrees

        return self.angle                                                   # Return encoder angle (0 to 359.97802734375)

    def readMagnitude(self):

        magnitude = self.bus.read_i2c_block_data(self.address, 0xFC, 2)     # Request data from registers 0xFC & 0xFD of the encoder
                                                                            # Takes ~700 microseconds.

        self.magnitude = (magnitude[0] << 6) | magnitude[1]                 # Remove unused bits 6 & 7 from byte 0xFD creating 14 bit value

        return self.magnitude                                               # Return encoder magnitude
