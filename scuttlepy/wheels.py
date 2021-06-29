#!/usr/bin/python3

import time
import numpy as np                                                          # for handling arrays

import logging
from scuttlepy import PID                                                   # for PID controller
from scuttlepy import motor                                                 # for controlling motors
from scuttlepy import encoder                                               # for reading encoders

from adafruit_platformdetect import Detector
detector = Detector()

class Wheel:

    def __init__(self, motor_output, encoder_address, wheel_radius=41, invert_motor=False, invert_encoder=False, KP=0.004, KI=0.025, KD=0, openLoop=False, debugging=False, debugFile=None):

        self.debugging = debugging
        self.debugFile = debugFile
        self.openLoop = openLoop

        if self.debugging:
            if self.debugFile:
                logging.basicConfig(filename=self.debugFile, format='%(asctime)s %(message)s', filemode='w')
                self.logger = logging.getself.logger()                                                # create an object
                self.logger.setLevel(logging.DEBUG)                                              # set threshold of self.logger to DEBUG
                self.logger.disabled = False
                self.t0 = time.monotonic()
            else:
                print('Please specify a debug file name!')
                self.debugging = False

        self.targetSpeed = 0                                                # (rad/s), use self.speed instead when possible!
        self.speed = 0                                                      # (rad/s), use self.speed instead when possible!
        self.radius = wheel_radius                                          # mm
        self.motor = motor.Motor(motor_output, invert=invert_motor)
        self.encoder = encoder.Encoder(encoder_address)
        self.invert_motor = invert_motor
        self.invert_encoder = invert_encoder

        self.positionInitial = 0
        self.positionFinal = 0

        self.pulleyRatio = 0.5                                              # pulley ratio = 0.5 wheel turns per pulley turn

        self.KP = KP
        self.KI = KI
        self.KD = KD

        self.pid = PID.PID(self.KP, self.KI, self.KD)
        # self.pid.setWindup(1)

        self.roll = 2 * np.pi / self.encoder.resolution
        self.gap = 0.5 * self.roll                                          # degrees specified as limit for rollover
        self.loopFreq = 50                                                  # Target Wheel Loop frequency (Hz)
        self.period = 1/self.loopFreq                                         # corrected wait time between encoder measurements (s)

        self.pid.setSampleTime(1/self.loopFreq)

    def getTravel(self, position0, position1):                              # calculate the increment of a wheel in radians
        if not self.invert_encoder:
            diff = position1 - position0                                    # take in the values in raw encoder position
            travel = diff                                                   # reset the travel reading
            if((-travel) >= self.gap):                                      # if movement is large (has rollover)
                travel = (diff + self.roll)                                 # handle forward rollover
            if(travel >= self.gap):
                travel = (diff - self.roll)                                 # handle reverse rollover
        elif self.invert_encoder:
            diff = position0 - position1                                    # DUPLICATE CODE
            travel = diff
            if((-travel) >= self.gap):
                travel = (diff + self.roll)
            if(travel >= self.gap):
                travel = (diff - self.roll)

        travel = travel * self.encoder.resolution                           # go from raw value to radians
        travel = travel * self.pulleyRatio                                  # go from motor pulley to wheel pulley
        return travel                                                       # return in radians of wheel advancement

    def getAngularVelocity(self):                                           # Use self.speed instead when possible!

        initialPosition = self.encoder.readPos()
        initialTime = time.monotonic()                                      # time.monotonic() reports in seconds
        time.sleep(self.period)                                               # delay specified amount
        finalPosition = self.encoder.readPos()
        finalTime = time.monotonic()
        deltaTime = round((finalTime - initialTime), 3)                     # new scalar delta time value

        travel = self.getTravel(initialPosition, finalPosition)             # movement calculations

        self.speed = round(travel / deltaTime, 3)                           # speed produced from true wheel travel (rad)

        if self.debugging:
            myTime = round(time.monotonic() - self.t0,3)
            self.logger.debug("Wheel_speed(rad/s) " + str(round(self.speed, 3)) +
                " timeStamp " + str(myTime) )

        return self.speed                                                   # returns wheel velocity in radians/second

    def setAngularVelocity(self, angularVelocity):
        self.targetSpeed = angularVelocity

        if not self.openLoop:
            self.pid.SetPoint = self.targetSpeed
            self.speed = self.getAngularVelocity()
            self.pid.update(self.speed)
            duty = self.pid.output

            if self.debugging:
                myTime = round(time.monotonic() - self.t0,3)
                self.logger.debug(" timeStamp " + str(myTime) +
                    " u_p " + str(round(self.pid.PTerm, 3)) +
                    " u_total " + str(round (self.pid.output,2) ) )

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


if __name__ == "__main__":

    if detector.board.BEAGLEBONE_BLUE:

        l_wheel = Wheel(1, 0x40, invert_encoder=True)                           # Left Motor  (ch1)
        r_wheel = Wheel(2, 0x41) 	                                            # Right Motor (ch2)

    elif detector.board.RASPBERRY_PI_40_PIN or detector.board.JETSON_NANO:

        l_wheel = Wheel((32,29), 0x40, invert_encoder=True)                     # Left Motor  (ch1)
        r_wheel = Wheel((33,31), 0x41) 	                                        # Right Motor (ch2)

    try:
        while True:

            r_wheel.setAngularVelocity(np.pi)
            l_wheel.setAngularVelocity(np.pi)

            print(l_wheel.getAngularVelocity(), ' rad/s\t', r_wheel.getAngularVelocity(), ' rad/s')

    finally:
        r_wheel.stop()
        l_wheel.stop()
