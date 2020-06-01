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

    def readAngle(self):

        # The AS5048B encoder gives a 14 bit angular reading
        self.msB = self.bus.read_byte_data(self.address, 0xFE)    # capture the 8 msb's from encoder
        self.lsB = self.bus.read_byte_data(self.address, 0xFF)    # capture the 6 lsb's from encoder

        self.angle = (self.msB << 6) | self.lsB
        self.angle = self.angle * (360 / 2**14)      # scale values to get degrees
        return self.angle

    def readMagnitude(self):

        # The AS5048B encoder gives a 14 bit angular reading
        self.msB = self.bus.read_byte_data(self.address, 0xFC)    # capture the 8 msb's from encoder
        self.lsB = self.bus.read_byte_data(self.address, 0xFD)    # capture the 6 lsb's from encoder

        self.magnitude = (self.msB << 6) | self.lsB

        return self.magnitude


if __name__ == "__main__":

    leftEncoder =   Encoder(0x43)
    rightEncoder =  Encoder(0x40)

    while 1:
        # leftAngle = round(leftEncoder.readAngle(), 2)
        # rightAngle = round(rightEncoder.readAngle(), 2)

        leftMag = round(leftEncoder.readMagnitude(), 2)
        rightMag = round(rightEncoder.readMagnitude(), 2)

        # print(leftAngle, "\t", rightAngle)
        print(leftMag, "\t", rightMag)

        time.sleep(0.01)