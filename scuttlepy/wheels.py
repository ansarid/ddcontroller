#!/usr/bin/python3

import time
import numpy as np

import scuttlepy.PID as PID
import scuttlepy.motor as motor
import scuttlepy.encoder as encoder
from scuttlepy.constants import *

class Wheel:

    def __init__(self, motor_output, encoder_address, wheel_radius=0.04165, invert_motor=False, invert_encoder=False, KP=0.004, KI=0.025, KD=0, openLoop=False):

        self.openLoop = openLoop

        self.targetSpeed = 0                                            # (rad/s), use self.speed instead when possible!
        self.speed = 0                                                  # (rad/s), use self.speed instead when possible!
        self.radius = wheel_radius                                      # m
        self.motor = motor.Motor(motor_output, invert=invert_motor)     # Create motor object
        self.encoder = encoder.Encoder(encoder_address, bus=I2C_BUS)
        self.invert_motor = invert_motor
        self.invert_encoder = invert_encoder

        self.pulleyRatio = 0.5                                          # pulley ratio = 0.5 wheel turns per pulley turn

        self.KP = KP
        self.KI = KI
        self.KD = KD

        self.pid = PID.PID(self.KP, self.KI, self.KD)
        # self.pid.setWindup(1)

        self.roll = 2 * np.pi / self.encoder.resolution
        self.gap = 0.5 * self.roll                                      # degrees specified as limit for rollover
        self.loopFreq = 50                                              # Target Wheel Loop frequency (Hz)
        self.period = 1/self.loopFreq                                   # corrected wait time between encoder measurements (s)

        self._positionFinal = 0
        self._positionInitial = 0

        # self.pid.setSampleTime(1/self.loopFreq)

    def getRotation(self, position0, position1):                        # calculate the increment of a wheel in ticks
        if not self.invert_encoder:
            changeInRotation = position1 - position0                    # take in the values in raw encoder position
        elif self.invert_encoder:
            changeInRotation = position0 - position1
        rotation = changeInRotation                                     # reset the rotation reading
        if(-rotation >= self.gap):                                      # if movement is large (has rollover)
            rotation = (changeInRotation + self.roll)                   # handle forward rollover
        if(rotation >= self.gap):
            rotation = (changeInRotation - self.roll)                   # handle reverse rollover

        rotation = rotation * self.encoder.resolution                   # go from raw value to radians
        rotation = rotation * self.pulleyRatio                          # go from motor pulley to wheel pulley
        return rotation                                                 # return wheel advancement in ticks

    def getTravel(self):                                      # calculate travel of the wheel in meters
        initialPosition = self.encoder.readPos()
        initialTime = time.monotonic_ns()                               # time.monotonic_ns() reports in nanoseconds
        time.sleep(self.period)                                         # delay specified amount
        finalPosition = self.encoder.readPos()
        finalTime = time.monotonic_ns()
        deltaTime = (finalTime - initialTime)/1e9                       # new scalar delta time valuel, convert ns to s
        rotation = self.getRotation(initialPosition, finalPosition)     # movement calculations

        self.speed = (rotation / deltaTime)                             # speed produced from true wheel rotation (rad)

        D = (2 * np.pi * self.radius)*(rotation/self.encoder.resolution)
        return D

    def getAngularVelocity(self):                                       # Use self.speed instead when possible!
        self.getTravel()
        return self.speed                                               # returns wheel velocity in radians/second

    def setAngularVelocity(self, angularVelocity):
        self.targetSpeed = angularVelocity

        if not self.openLoop:
            self.pid.SetPoint = self.targetSpeed
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
            if self.targetSpeed > 0.15:
                duty = (0.098*self.targetSpeed)+0.148
            elif self.targetSpeed < -0.15:
                duty = (0.098*self.targetSpeed)-0.148
            else:
                duty = 0

        self.motor.setDuty(duty)

    def stop(self):
        self.motor.stop()
