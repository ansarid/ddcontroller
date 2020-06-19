#!/usr/bin/python3

# speed_control.py takes target speeds and generates duty cycles
# to send to motors, and has a function to execute PID control.

# Import external libraries
import time
import math
import numpy as np                                            # for handling arrays

# Import local files

from scuttlepy import encoder                                   # for reading encoders
from scuttlepy import PID                                       # for PID controller
from scuttlepy import motor                                     # for controlling motors


class Wheel:

    def __init__(self, motor_channel, encoder_address, wheel_radius=41, invert_motor=False, invert_encoder=False, KP=0.07, KI=1.15, KD=0):
        self.speed = 0                                          # (rad/s)
        self.radius = wheel_radius                              # mm
        self.motor = motor.Motor(motor_channel, invert=invert_motor)
        self.encoder = encoder.Encoder(encoder_address)
        self.invert_motor = invert_motor
        self.invert_encoder = invert_encoder

        self.positionInitial = 0
        self.positionFinal = 0

        self.pulleyRatio = 0.5                                  # pulley ratio = 0.5 wheel turns per pulley turn

        self.KP = KP
        self.KI = KI
        self.KD = KD

        self.pid = PID.PID(self.KP, self.KI, self.KD)
        # self.pid.setWindup(1)

        self.roll = 2 * math.pi / self.encoder.resolution
        self.gap = 0.5 * self.roll                              # degrees specified as limit for rollover
        self.wait = 0.02                                        # wait time between encoder measurements (s)

        # self.pid.setSampleTime(self.wait)

    def getTravel(self, position0, position1):                  # calculate the increment of a wheel in radians
        diff = position1 - position0                            # take in the values in raw encoder position
        if not self.invert_encoder:
            travel = diff                                       # reset the travel reading
            if((-travel) >= self.gap):                          # if movement is large (has rollover)
                travel = (diff + self.roll)                     # handle forward rollover
            if(travel >= self.gap):
                travel = (diff - self.roll)                     # handle reverse rollover
        else:
            diff = position0 - position1
            travel = diff
            if((-travel) >= self.gap):
                travel = (diff + self.roll)
            if(travel >= self.gap):
                travel = (diff - self.roll)

        travel = travel * self.encoder.resolution               # go from raw value to radians
        travel = travel * self.pulleyRatio                      # go from motor pulley to wheel pulley
        # print(travel)
        return(travel)                                          # return in radians of wheel advancement

    def getAngularVelocity(self):

        initialPosition = self.encoder.readPos()
        initialTime = time.time()                               # time.time() reports in seconds
        time.sleep(self.wait)                                   # delay specified amount
        finalPosition = self.encoder.readPos()
        finalTime = time.time()
        deltaTime = round((finalTime - initialTime), 3)         # new scalar delta time value

        # ---- movement calculations
        travel = self.getTravel(initialPosition, finalPosition)

        self.speed = round((travel * self.pulleyRatio) / deltaTime, 3)
        return self.speed                                       # returns pdc in radians/second

    def setAngularVelocity(self, pdt):
        self.pid.SetPoint = pdt
        self.speed = self.getAngularVelocity()
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
        duty = sorted([-1, duty, 1])[1]               # place bounds on the motor commands
        print("motorDuty:", round(duty,2))
        self.motor.setDuty(round(duty,2))                      # must round to ensure driver handling!


if __name__ == "__main__":

    r_wheel = Wheel(2, 0x40) 	                                # Right Motor (ch2)
    l_wheel = Wheel(1, 0x43, invert_encoder=True)               # Left Motor  (ch1)

    print("Left Wheel, Right Wheel")

    speeds = []

    try:

        while True:

            # print(l_wheel.getAngularVelocity(), ",", r_wheel.getAngularVelocity())

            # Set Wheel Speed to 6.28 rad/s
            # l_wheel.setAngularVelocity(3.14)
            # r_wheel.setAngularVelocity(3.14)

            l_wheel.motor.setDuty(0.7)
            # r_wheel.motor.setDuty(-0.7)

            l_angle = round(l_wheel.getAngularVelocity(),2)
            speeds.append(l_angle)
            # r_angle = round(r_wheel.getAngularVelocity(),2)
            print(l_angle)

    except KeyboardInterrupt:

        print("Average: ", np.average(speeds))