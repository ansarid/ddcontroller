#!/usr/bin/python3

# speed_control.py takes target speeds and generates duty cycles
# to send to motors, and has a function to execute PID control.

# Import external libraries

import time
import math
import numpy as np                                                          # for handling arrays
import logging

# Import local files

from scuttlepy import PID                                                   # for PID controller
from scuttlepy import motor                                                 # for controlling motors
from scuttlepy import encoder                                               # for reading encoders

# Create and configure logger
# logging.basicConfig(filename="wheelsTest.log", format='%(asctime)s %(message)s', filemode='w')
# logger = logging.getLogger()                                                # create an object
# logger.setLevel(logging.DEBUG)                                              # set threshold of logger to DEBUG
# logger.disabled = True

# logger.debug("ColumnA ColumnB ColumnC ColumnD")


class Wheel:

    def __init__(self, motor_channel, encoder_address, wheel_radius=41, invert_motor=False, invert_encoder=False, KP=0.05, KI=0.06, KD=0):

        self.speed = 0                                                      # (rad/s), use self.speed instead when possible!
        self.radius = wheel_radius                                          # mm
        self.motor = motor.Motor(motor_channel, invert=invert_motor)
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

        self.roll = 2 * math.pi / self.encoder.resolution
        self.gap = 0.5 * self.roll                                          # degrees specified as limit for rollover
        self.wait = 0.02                                                    # wait time between encoder measurements (s)

        # self.pid.setSampleTime(self.wait)

    def getTravel(self, position0, position1):                              # calculate the increment of a wheel in radians
        diff = position1 - position0                                        # take in the values in raw encoder position
        if not self.invert_encoder:
            travel = diff                                                   # reset the travel reading
            if((-travel) >= self.gap):                                      # if movement is large (has rollover)
                travel = (diff + self.roll)                                 # handle forward rollover
            if(travel >= self.gap):
                travel = (diff - self.roll)                                 # handle reverse rollover
        else:
            diff = position0 - position1
            travel = diff
            if((-travel) >= self.gap):
                travel = (diff + self.roll)
            if(travel >= self.gap):
                travel = (diff - self.roll)

        travel = travel * self.encoder.resolution                           # go from raw value to radians
        travel = travel * self.pulleyRatio                                  # go from motor pulley to wheel pulley
        return(travel)                                                      # return in radians of wheel advancement

    def getAngularVelocity(self):                                           # Use self.speed instead when possible!

        initialPosition = self.encoder.readPos()
        initialTime = time.time()                                           # time.time() reports in seconds
        time.sleep(self.wait)                                               # delay specified amount
        finalPosition = self.encoder.readPos()
        finalTime = time.time()
        deltaTime = round((finalTime - initialTime), 3)                     # new scalar delta time value

        travel = self.getTravel(initialPosition, finalPosition)             # movement calculations

        self.speed = round(travel / deltaTime, 3)                           # speed produced from true wheel travel (rad)
        # logger.debug("Wheel_speed(rad/s) " + str(round(self.speed, 3)) +
        #     " timeStamp " + str(time.monotonic()) )
        return self.speed                                                   # returns pdc in radians/second

    def setAngularVelocity(self, phiDotTarget):
        self.pid.SetPoint = phiDotTarget
        # self.speed = self.getAngularVelocity()  # no update required - speed updated in navigation program
        self.pid.update(self.speed)
        duty = self.pid.output

        ### THIS NEEDS TO BE REFACTORED ###
        if -0.222 < duty and duty < 0.222:
            duty = (duty * 3)
        elif duty > 0.222:
            duty = ((duty * 0.778) + 0.222)
        else:
            duty = ((duty * 0.778) - 0.222)
        ### THIS NEEDS TO BE REFACTORED ###

        duty = sorted([-1, duty, 1])[1]                                     # place bounds on the motor commands
        self.motor.setDuty(round(duty, 2))                                  # must round to ensure driver handling!


if __name__ == "__main__":

    r_wheel = Wheel(2, 0x41) 	                                            # Right Motor (ch2)
    l_wheel = Wheel(1, 0x40, invert_encoder=True)                           # Left Motor  (ch1)

    print("Left Wheel, Right Wheel")

    startTime = time.time()

    try:

        while (time.time() - startTime) < 4:

            print(l_wheel.speed, ",", r_wheel.speed)
            l_wheel.setAngularVelocity(4.68)
            r_wheel.setAngularVelocity(4.68)

            time.sleep(0.060)

        while (time.time() - startTime) > 4:

            print(l_wheel.speed, ",", r_wheel.speed)
            l_wheel.setAngularVelocity(2.92)
            r_wheel.setAngularVelocity(2.92)

            time.sleep(0.060)

    except KeyboardInterrupt:
        exit()
