#!/usr/bin/python3

from os import getrandom
import time
import numpy as np
from collections import deque

import scuttlepy.PID as PID
import scuttlepy.motor as motor
import scuttlepy.encoder as encoder
from scuttlepy.constants import *

class Wheel:

    def __init__(self, motorOutput, encoderAddress, wheelRadius=0.04165, invertMotor=False, invertEncoder=False, KP=0.004, KI=0.025, KD=0, openLoop=False):

        self.openLoop = openLoop
        self.invertMotor = invertMotor                                  # Invert motor mode
        self.invertEncoder = invertEncoder                              # Invert encoder mode

        self.motor = motor.Motor(motorOutput, invert=invertMotor)       # Create motor object
        self.encoder = encoder.Encoder(encoderAddress, bus=I2C_BUS)     # Create encoder object

        self.radius = wheelRadius                                       # meters
        self.pulleyRatio = 15/30                                        # number of teeth on motor pulley / number of teeth on wheel pulley

        self.position = self.encoder.readPosition()                     # Raw encoder 'ticks'
        self.timestamp = time.monotonic_ns()                            # Stores age of data
        self.targetVelocity = 0                                         # (radians/second)
        self.angularVelocity = 0                                        # (radians/second)
        self.linearVelocity = 0                                         # (meters/second)

        # PID Gains
        self.KP = KP                                                    # PID controller P gain
        self.KI = KI                                                    # PID controller I gain
        self.KD = KD                                                    # PID controller D gain

        self.pid = PID.PID(self.KP, self.KI, self.KD)                   # Create PID controller object

        self.rolloverLimit = self.pulleyRatio * self.encoder.resolution # limit for rollover
        self._positions = deque([self.position, self.encoder.readPosition()], maxlen=2)
        self._timestamps = deque([time.monotonic_ns()]*2, maxlen=2)

        self.update()

    def update(self):
        self._positions.append(self.encoder.readPosition())
        self._timestamps.append(time.monotonic_ns())

        self.position = self._positions[1]
        self.timestamp = self._timestamps[1]

        # self.angularVelocity = self.getAngularVelocity()
        print(self.getAngularVelocity())

    def getRotation(self):                                              # calculate the increment of a wheel in ticks

        rotation = self._positions[1] - self._positions[0]              # calculate how much wheel has rotated

        if(-rotation >= self.rolloverLimit):                            # if movement is large (has rollover)
            rotation = (rotation + self.encoder.resolution)             # handle forward rollover
        if(rotation >= self.rolloverLimit):
            rotation = (rotation - self.encoder.resolution)             # handle reverse rollover

        rotation *= self.pulleyRatio                                    # go from motor pulley to wheel pulley

        return rotation                                                 # return wheel advancement in ticks

    def getTravel(self):                                                # calculate travel of the wheel in meters

        rotation = self.getRotation()                                                   # get wheel rotation between measurements
        distance = (2*np.pi*self.radius)*(rotation/self.encoder.resolution)             # calculate distance traveled in wheel rotation

        return distance

    def getLinearVelocity(self):                                                        # get wheel linear velocity

        distance = self.getTravel()                                                     # get wheel travel
        deltaTime = (self._timestamps[1] - self._timestamps[0])/1e9                     # calculate deltaTime, convert from ns to s
        self.linearVelocity = distance/deltaTime                                        # calculate wheel linear velocity
        return self.linearVelocity                                                      # return wheel linear velocity in meters/second


    def getAngularVelocity(self):                                                       # get wheel angular velocity

        rotation = self.getRotation()                                                                       # get wheel rotation between measurements
        deltaTime = (self._timestamps[1] - self._timestamps[0])/1e9                                         # calculate deltaTime, convert from ns to s
        self.angularVelocity = ((rotation * (np.pi/(self.encoder.resolution/2))) / deltaTime)               # speed produced from true wheel rotation (rad)
        return self.angularVelocity                                                                         # returns wheel angular velocity in radians/second

    def setAngularVelocity(self, angularVelocity):
        self.targetAngularVelocity = angularVelocity
        self.update()

        if not self.openLoop:
            self.pid.SetPoint = self.targetAngularVelocity
            self.speed = self.getAngularVelocity()
            self.pid.update(self.speed)
            duty = self.pid.output

            ### THIS NEEDS TO BE REFACTORED ###
            if -0.222 < duty and duty < 0.222:
                duty = (duty * 3)
            elif duty >= 0.222:
                duty = 0.666 + (0.429*(duty-0.222))
            else:
                duty = -0.666 + (-0.429*(duty+0.222))
            ### THIS NEEDS TO BE REFACTORED ###

        else:       # VERY TEMPORARY CODE
            if self.targetAngularVelocity > 0.15:
                duty = (0.098*self.targetAngularVelocity)+0.148
            elif self.targetAngularVelocity < -0.15:
                duty = (0.098*self.targetAngularVelocity)-0.148
            else:
                duty = 0

        self.motor.setDuty(duty)

    def stop(self):
        self.motor.stop()

# TESTING

if __name__ == "__main__":

    # leftWheel = Wheel((11,12), 0x43, invertEncoder=True, openLoop=True)	# Create Left Wheel Object
    rightWheel = Wheel((15,16), 0x41, openLoop=True) 	               	    # Create Right Wheel Object

    try:

        # rightWheel.motor.setDuty(1)
        rightWheel.setAngularVelocity(2*np.pi)

        while True:

            rightWheel.update()
            # leftWheel.setAngularVelocity(2*np.pi)
            # print(round(leftWheel.getAngularVelocity(),3), ' rad/s\t', round(rightWheel.getAngularVelocity(), 3), ' rad/s')
            # print(rightWheel.getTravel())
            # print(rightWheel.getRotation())
            # print(rightWheel.encoder.readPosition())
            # print(rightWheel._timestamps, rightWheel._positions)

            time.sleep(1/50)

    except KeyboardInterrupt:

        print('Stopping...')

    finally:

        # leftWheel.stop()
        rightWheel.stop()
        print('Stopped.')
