#!/usr/bin/python3

# This example reads left & right encoders & outputs the position (deg) to the terminal
# Left has address 40 & right has 41
# Code for Beagle Hardware

# from adafruit_platformdetect import Detector
# detector = Detector()
# if detector.board.BEAGLEBONE_BLUE:

# Import external libraries
import time
import math
from smbus2 import SMBus


class Encoder:

    def __init__(self, address, bus=1, invert=False):

        self.bus = SMBus(bus)
        self.address = address
        self.invert = invert
        self.resolution = ((math.pi*2)/2**14)                               # Define encoder angular resolution.
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

        magnitude = self.bus.read_i2c_block_data(self.address, 0xFC, 2)     # Request data from registers 0xFE & 0xFF of the encoder
                                                                            # Takes ~700 microseconds.

        self.magnitude = (magnitude[0] << 6) | magnitude[1]                 # Remove unused bits 6 & 7 from byte 0xFF creating 14 bit value

        return self.magnitude                                               # Return encoder magnitude


if __name__ == "__main__":

    rightEncoder = Encoder(0x40)                                            # Create encoder object for right encoder on address 0x40
    leftEncoder = Encoder(0x43, invert=True)                                # Create encoder object for left encoder on address 0x43

    while True:

        rightPos = round(rightEncoder.readPos(), 2)
        leftPos = round(leftEncoder.readPos(), 2)

        rightAngle = round(rightEncoder.readAngle(), 2)
        leftAngle = round(leftEncoder.readAngle(), 2)

        rightMag = round(rightEncoder.readMagnitude(), 2)
        leftMag = round(leftEncoder.readMagnitude(), 2)

        # print(leftPos, "\t", rightPos)
        print(leftAngle, "\t", rightAngle)
        # print(leftMag, "\t", rightMag)

        time.sleep(0.01)
