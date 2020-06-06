#!/usr/bin/python3

# This example reads left and right encoders and outputs the position (deg) to the terminal
# Left has address 40 and right has 41
# Code for Beagle Hardware

# Import internal libraries
# from scuttlepy.L1 import log

# Import external libraries
import time
import numpy as np                          # for handling arrays
from smbus2 import SMBus


class Encoder:

    def __init__(self, address, bus=1, invert=False):

        self.bus = SMBus(bus)
        self.address = address
        self.invert = invert
        self.res = (360/2**14)

    def readPos(self):

        # The AS5048B encoder gives a 14 bit angular reading
        pos = self.bus.read_i2c_block_data(self.address, 0xFE, 2)  # Takes ~700 microseconds.

        if not self.invert:
            self.position = (pos[0] << 6) | pos[1]
        else:
            self.position = (2**14)-((pos[0] << 6) | pos[1])

        return self.position

    def readAngle(self):

        # The AS5048B encoder gives a 14 bit angular reading
        angle = self.bus.read_i2c_block_data(self.address, 0xFE, 2)

        angle = (angle[0] << 6) | angle[1]

        if not self.invert:
            self.angle = angle * (360 / 2**14)              # scale values to get degrees
        else:
            self.angle = 360 - (angle * (360 / 2**14))      # scale values to get degrees

        return self.angle

    def readMagnitude(self):

        # The AS5048B encoder gives a 14 bit angular reading
        magnitude = self.bus.read_i2c_block_data(self.address, 0xFC, 2)

        self.magnitude = (magnitude[0] << 6) | magnitude[1]

        return self.magnitude


if __name__ == "__main__":

    rightEncoder =  Encoder(0x40)
    leftEncoder =   Encoder(0x43, invert=True)

    while True:

        rightAngle = round(rightEncoder.readAngle(), 2)
        leftAngle = round(leftEncoder.readAngle(), 2)

        rightMag = round(rightEncoder.readMagnitude(), 2)
        leftMag = round(leftEncoder.readMagnitude(), 2)

        print(leftAngle, "\t", rightAngle)
        # print(leftMag, "\t", rightMag)

        time.sleep(0.01)
