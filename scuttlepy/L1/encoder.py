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

    def __init__(self, address, bus=1):

        self.bus = SMBus(bus)
        self.address = address

    def readPos(self):

        # The AS5048B encoder gives a 14 bit angular reading
        self.pos = self.bus.read_i2c_block_data(self.address, 0xFE, 2)  # Takes ~700 microseconds.

        self.pos = (self.pos[0] << 6) | self.pos[1]

        return self.pos

    def readAngle(self):

        # The AS5048B encoder gives a 14 bit angular reading
        self.angle = self.bus.read_i2c_block_data(self.address, 0xFE, 2)

        self.angle = (self.msB << 6) | self.lsB
        self.angle = self.angle * (360 / 2**14)      # scale values to get degrees

        return self.angle

    def readMagnitude(self):

        # The AS5048B encoder gives a 14 bit angular reading
        self.magnitude = self.bus.read_i2c_block_data(self.address, 0xFC, 2)

        self.magnitude = (self.msB << 6) | self.lsB

        return self.magnitude


if __name__ == "__main__":

    leftEncoder =   Encoder(0x43)
    rightEncoder =  Encoder(0x40)

    while True:

        leftAngle = round(leftEncoder.readAngle(), 2)
        rightAngle = round(rightEncoder.readAngle(), 2)

        leftMag = round(leftEncoder.readMagnitude(), 2)
        rightMag = round(rightEncoder.readMagnitude(), 2)

        print(leftAngle, "\t", rightAngle)
        print(leftMag, "\t", rightMag)

        time.sleep(0.01)
