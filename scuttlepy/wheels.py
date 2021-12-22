#!/usr/bin/python3

import time
import numpy as np
from numpy.lib.ufunclike import _dispatcher

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
        self.encoder = encoder.Encoder(encoder_address, bus=I2C_BUS)    # Create encoder object
        self.invert_motor = invert_motor                                # Invert motor mode
        self.invert_encoder = invert_encoder                            # Invert encoder mode

        self.pulleyRatio = 0.5                                          # pulley ratio = 0.5 wheel turns per pulley turn

        self.KP = KP
        self.KI = KI
        self.KD = KD

        self.pid = PID.PID(self.KP, self.KI, self.KD)                   # Create PID controller object
        # self.pid.setWindup(1)

        self.gap = 0.5 * self.encoder.resolution                        # limit for rollover
        self.loopFreq = 50                                              # Target Wheel Loop frequency (Hz)
        self.loopFrequencyAdjustment = 0                                # Adjusted Wheel Loop frequency (Hz)
        self.period = 1/self.loopFreq                                   # corrected wait time between encoder measurements (s)

        self._positionInitial = 0
        self._positionFinal = 0

        # self.pid.setSampleTime(1/self.loopFreq)

    def getRotation(self, position0, position1):                        # calculate the increment of a wheel in ticks
        if not self.invert_encoder:
            changeInRotation = position1 - position0                    # take in the values in raw encoder position
        elif self.invert_encoder:
            changeInRotation = position0 - position1
        rotation = changeInRotation                                     # reset the rotation reading
        if(-rotation >= self.gap):                                      # if movement is large (has rollover)
            rotation = (changeInRotation + self.encoder.resolution)     # handle forward rollover
        if(rotation >= self.gap):
            rotation = (changeInRotation - self.encoder.resolution)     # handle reverse rollover

        # rotation = rotation * self.pulleyRatio                          # go from motor pulley to wheel pulley
        return rotation                                                 # return wheel advancement in ticks

    def getTravel(self):                                                # calculate travel of the wheel in meters
        initialPosition = self.encoder.readPos()                        # read encoder initial position
        initialTime = time.monotonic_ns()                               # get time at initial position reading
        time.sleep(self.period)                                         # delay specified amount of time (FIX THIS)
        finalPosition = self.encoder.readPos()                          # read encoder final position

        finalTime = time.monotonic_ns()                                 # get time at final position reading
        deltaTime = (finalTime - initialTime)/1e9                       # new scalar delta time value, convert ns to s
        self.loopFrequencyAdjustment = deltaTime - self.period

        # print(self.period*1000, '\t', deltaTime*1000, '\t', self.loopFrequencyAdjustment*1000)

        rotation = self.getRotation(initialPosition, finalPosition)     # movement calculations

        self.speed = ((rotation * (np.pi/(self.encoder.resolution/2))) / deltaTime)     # speed produced from true wheel rotation (rad)
        distance = (2*np.pi*self.radius)*(rotation/self.encoder.resolution)             # calculate distance traveled in wheel rotation

        return distance

    def getAngularVelocity(self):                                       # Use self.speed instead when possible!
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
