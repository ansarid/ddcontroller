#!/usr/bin/python3

# speed_control.py takes target speeds and generates duty cycles
# to send to motors, and has a function to execute PID control.

# Import external libraries
import time
import math
# import numpy as np                                            # for handling arrays

# Import local files

from scuttlepy.perception import encoder                        # for reading encoders
from scuttlepy.control import PID                               # for PID controller
from scuttlepy.actuation import motor                           # for controlling motors


class Wheel:

    def __init__(self, motor_channel, encoder_address, wheel_radius=41, invert_motor=False, invert_encoder=False):
        self.speed = 0                                          # (rad/s)
        self.radius = wheel_radius                              # mm
        self.motor = motor.Motor(motor_channel, invert=invert_motor)
        self.encoder = encoder.Encoder(encoder_address, invert=invert_encoder)
        self.invert_motor = invert_motor
        self.invert_encoder = invert_encoder

        # self.pid = PID.PID(0.06, 0.9, 0.000009)
        self.pid = PID.PID(0.04, 0.04, 0.0)

        self.pdCurrents = 0

        self.roll = int(360/self.encoder.res)                   # variable for rollover logic
        self.gap = 0.5 * self.roll                              # degrees specified as limit for rollover
        self.wait = 0.02                                        # wait time between encoder measurements (s)

    def getTravel(self, position0, position1):                            # calculate the delta on Left wheel
        travel = position1 - position0                                    # reset the travel reading
        if((-travel) >= self.gap):                              # if movement is large (has rollover)
            travel = (position1 - position0 + self.roll)                  # handle forward rollover
        if(travel >= self.gap):
            travel = (position1 - position0 - self.roll)                    # handle reverse rollover
        return(travel)

    def getSpeed(self):
        encoder_deg = self.encoder.readPos()                    # grabs the current encoder readings in integer values
        position0 = round(encoder_deg, 1)                            # reading in degrees.
        time1 = time.time()                                        # time.time() reports in seconds
        time.sleep(self.wait)                                   # delay specified amount
        encoder_deg = self.encoder.readPos()                    # grabs the current encoder readings in integer values
        position1 = round(encoder_deg, 1)                            # reading in degrees.
        time2 = time.time()                                        # reading about .003 seconds
        deltaT = round((time2 - time1), 3)                            # new scalar dt value

        # ---- movement calculations
        travel = self.getTravel(position0, position1) * self.encoder.res           # grabs travel of left wheel, degrees

        # build an array of wheel speeds in rad/s
        travel = travel * 0.5                                       # pulley ratio = 0.5 wheel turns per pulley turn
        travel = math.radians(travel)                               # convert degrees to radians
        travel = round(travel, 3)                                   # round the array
        wheelSpeed = travel / deltaT
        wheelSpeed = round(wheelSpeed, 3)
        self.speed = wheelSpeed
        return self.speed                                       # returns current phi dot in radians/second

    def setSpeed(self, pdt):
        self.pid.SetPoint = pdt
        self.speed = self.getSpeed()
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

        self.motor.setDuty(duty)


if __name__ == "__main__":

    r_wheel = Wheel(2, 0x40) 	                                # Right Motor (ch2)
    l_wheel = Wheel(1, 0x43, invert_encoder=True)                       # Left Motor  (ch1)

    print("Left Wheel, Right Wheel")

    while True:

        print(l_wheel.getSpeed(), ",", r_wheel.getSpeed())

        # Set Wheel Speed to 6.28 rad/s
        r_wheel.setSpeed(6.28)
        l_wheel.setSpeed(6.28)
