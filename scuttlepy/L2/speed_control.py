#!/usr/bin/python3

# speed_control.py takes target speeds and generates duty cycles
# to send to motors, and has a function to execute PID control.

# Import external libraries
import time
import math
# import numpy as np                                  # for handling arrays

# Import local files

import scuttlepy.L1.motor as motor                  # for controlling motors
import scuttlepy.L1.encoder as encoder              # for reading encoders
import scuttlepy.L2.PID as PID                      # for PID controller

class Wheel:

    def __init__(self, motor_channel, encoder_address, wheel_radius=41):
        self.speed = 0                                      # (rad/s)
        self.radius = wheel_radius                          # mm
        self.motor = motor.Motor(motor_channel)
        self.encoder = encoder.Encoder(encoder_address)

        self.pid = PID.PID(0.06, 0.9, 0.000009)

        self.pdCurrents = 0

        self.res = (360/2**14)                          # resolution of the encoders
        self.roll = int(360/self.res)                   # variable for rollover logic
        self.gap = 0.5 * self.roll                      # degress specified as limit for rollover
        self.wait = 0.02                                # wait time between encoder measurements (s)

    def _getTravel(self, pos0, pos1):                    # calculate the delta on Left wheel
        trav = pos1 - pos0                              # reset the travel reading
        if((-trav) >= self.gap):                        # if movement is large (has rollover)
            trav = (pos1 - pos0 + self.roll)            # forward rollover
        if(trav >= self.gap):
            trav = (pos1 - pos0 - self.roll)            # reverse rollover
        return(trav)

    def getAngularVelocity(self):
        encoder_deg = self.encoder.readPos()            # grabs the current encoder readings in integer values
        pos0 = round(encoder_deg, 1)                    # reading in degrees.
        t1 = time.time()                                # time.time() reports in seconds
        time.sleep(self.wait)                           # delay specified amount
        encoder_deg = self.encoder.readPos()            # grabs the current encoder readings in integer values
        pos1 = round(encoder_deg, 1)                    # reading in degrees.
        t2 = time.time()                                # reading about .003 seconds
        deltaT = round((t2 - t1), 3)                    # new scalar dt value

        # ---- movement calculations
        trav = self._getTravel(pos0, pos1) * self.res     # grabs travel of left wheel, degrees
        # trav = self._getTravel(pos0, pos1)                  # grabs travel of left wheel, degrees
        # travL = -1 * travL                                # this wheel is inverted from the right side

        # build an array of wheel speeds in rad/s
        trav1 = trav
        trav = trav * 0.5                             # pulley ratio = 0.5 wheel turns per pulley turn
        trav = math.radians(trav)                     # convert degrees to radians
        trav = round(trav, 3)                         # round the array
        wheelSpeed = trav / deltaT
        wheelSpeed = round(wheelSpeed, 3)
        deg1 = round(pos1 * self.res, 1)
        deg0 = round(pos0 * self.res, 1)
        return(wheelSpeed)                              # returns pdc in radians/second

    def setSpeed(self, pdt):
        self.pid.SetPoint = pdt
        self.pid.update(self.getAngularVelocity())
        duty = self.pid.output

        if -0.222 < duty and duty < 0.222:
            duty = (duty * 3)
        elif duty > 0.222:
            duty = ((duty * 0.778) + 0.222)
        else:
            duty = ((duty * 0.778) - 0.222)

        # print("Duty: ",duty)
        self.motor.setDuty(duty)


if __name__ == "__main__":

    l_wheel = Wheel(1, 0x43) 	                        # Left Motor (ch1)
    r_wheel = Wheel(2, 0x40) 	                        # Right Motor (ch2)

    import numpy as np

    av = []

    # while True:
    for i in range(500):

        # print("Left Motor speed 1 rps")
        # l_wheel.setSpeed(math.pi*2)
        # r_wheel.setSpeed(math.pi*2)
        # time.sleep(10)

        # print("Left Motor speed 0.5 rps")
        r_wheel.setSpeed(math.radians(360))     # 6.283
        # r_wheel.setSpeed(3.14)
        # # time.sleep(10)

        # print("Left Motor speed 0 rps")
        # l_wheel.setSpeed(0)
        # r_wheel.setSpeed(0)
        # # time.sleep(10)

        av.append(r_wheel.getAngularVelocity())
        if len(av) > 50:
            av.pop(0)
        # r_wheel.motor.setDuty(0.7)
        print(r_wheel.getAngularVelocity(), ",", round(r_wheel.pid.last_error, 3), ",", round(np.average(av),2))
        # print(r_wheel.getAngularVelocity())
        # time.sleep(0.1)