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


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp                                            # proportional term
        self.ki = ki                                            # integral term
        self.kd = kd                                            # derivative term

        self.u_proportional = 0
        self.u_integral = 0
        self.u_derivative = 0

        self.t1 = time.time()
        self.e1 = 0

    def pid(self, pdt, pdc):    # phidottarget, phidotcurrent
        e0 = self.e1
        print("Error: ",e0)
        self.e1 = pdt - pdc
        t0 = self.t1
        self.t1 = time.time()
        dt = self.t1 - t0
        de_dt = (self.e1 - e0)/dt

        self.u_proportional = (self.e1 * self.kp)                                       # proportional term
        self.u_integral += (self.e1 * self.ki)                                          # integral term
        self.u_derivative = (de_dt * self.kd)                                      # derivative term

        u = (self.u_proportional + self.u_integral + self.u_derivative)

        return u


class Wheel:

    def __init__(self, motor_channel, encoder_address, wheel_radius=41):
        self.speed = 0                                      # (rad/s)
        self.radius = wheel_radius                          # mm
        self.motor = motor.Motor(motor_channel)
        self.encoder = encoder.Encoder(encoder_address)

        self.pid = PID(0.04, 0.04, 0.0)

        self.pdCurrents = 0

        self.res = (360/2**14)                          # resolution of the encoders
        self.roll = int(360/self.res)                   # variable for rollover logic
        self.gap = 0.5 * self.roll                      # degress specified as limit for rollover
        self.wait = 0.02                                # wait time between encoder measurements (s)

    def _getTravel(self, deg0, deg1):                    # calculate the delta on Left wheel
        trav = deg1 - deg0                              # reset the travel reading
        if((-trav) >= self.gap):                        # if movement is large (has rollover)
            trav = (deg1 - deg0 + self.roll)            # forward rollover
        if(trav >= self.gap):
            trav = (deg1 - deg0 - self.roll)            # reverse rollover
        return(trav)

    def _getPdCurrent(self):
        encoder_deg = self.encoder.readAngle()          # grabs the current encoder readings in degrees
        deg0 = round(encoder_deg, 1)                    # reading in degrees.
        t1 = time.time()                                # time.time() reports in seconds
        time.sleep(self.wait)                           # delay specified amount
        encoder_deg = self.encoder.readAngle()          # grabs the current encoder readings in degrees
        deg1 = round(encoder_deg, 1)                    # reading in degrees.
        t2 = time.time()                                # reading about .003 seconds
        deltaT = round((t2 - t1), 3)                    # new scalar dt value

        # ---- movement calculations
        trav = self._getTravel(deg0, deg1) * self.res      # grabs travel of left wheel, degrees
        # travL = -1 * travL                              # this wheel is inverted from the right side

        # build an array of wheel speeds in rad/s
        trav = trav * 0.5                             # pulley ratio = 0.5 wheel turns per pulley turn
        trav = math.radians(trav)                     # convert degrees to radians
        trav = round(trav, 3)                         # round the array
        wheelSpeed = trav / deltaT
        wheelSpeed = round(wheelSpeed, 3)
        return(wheelSpeed)                              # returns [pdl, pdr] in radians/second

    def setSpeed(self, pdt):
        pdc = self._getPdCurrent()                            # (rad/s)
        duty = self.pid.pid(pdt, pdc)

        if -0.222 < duty and duty < 0.222:
            duty = (duty * 3)
        elif duty > 0.222:
            duty = ((duty * 0.778) + 0.222)
        else:
            duty = ((duty * 0.778) - 0.222)

        print("duty: ",duty)
        self.motor.setDuty(duty)


if __name__ == "__main__":

    l_wheel = Wheel(1, 0x43) 	                        # Left Motor (ch1)
    r_wheel = Wheel(2, 0x40) 	                        # Right Motor (ch2)

    while True:

        print("Left Motor speed 1 rps")
        l_wheel.setSpeed(math.pi*2)
        r_wheel.setSpeed(math.pi*2)
