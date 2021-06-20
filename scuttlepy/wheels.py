#!/usr/bin/python3

# speed_control.py takes target speeds and generates duty cycles
# to send to motors, and has a function to execute PID control.

# Import external libraries

import time
import threading
import numpy as np                                                          # for handling arrays
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

    def __init__(self, motor_channel, encoder_address, wheel_radius=41, invert_motor=False, invert_encoder=False, KP=0.004, KI=0.025, KD=0):

        self.targetSpeed = 0                                                # (rad/s), use self.speed instead when possible!
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

        self.roll = 2 * np.pi / self.encoder.resolution
        self.gap = 0.5 * self.roll                                          # degrees specified as limit for rollover
        self.loopFreq = 50                                                  # Target Wheel Loop frequency (Hz)
        self.wait = 1/self.loopFreq                                         # corrected wait time between encoder measurements (s)

        # self.loopTime = self.wait
        # self.startTime = time.monotonic()

        self.pid.setSampleTime(1/self.loopFreq)

    # def _wheelLoop(self):
    #     while not self.stopped:

    #         self.setAngularVelocity(self.targetSpeed)                       # Set target velocity

    #         self.loopTime = (time.monotonic()-self.startTime)               # Calculate loop time
    #         loopTimeOffset = (1/self.loopFreq)-self.loopTime                # Calculate time difference between target and actaul loop time
    #         self.wait += loopTimeOffset                                     # Adjust wait time to achieve target
    #         self.startTime = time.monotonic()                               # reset startTime

    # def stop(self):
    #     self.stopped = True
    #     self.wheelThread.join()

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
        time.sleep(abs(self.wait))                                               # delay specified amount   ABS is probably the wrong fix
        finalPosition = self.encoder.readPos()
        finalTime = time.monotonic()
        deltaTime = round((finalTime - initialTime), 3)                     # new scalar delta time value

        travel = self.getTravel(initialPosition, finalPosition)             # movement calculations

        self.speed = round(travel / deltaTime, 3)                           # speed produced from true wheel travel (rad)
        # logger.debug("Wheel_speed(rad/s) " + str(round(self.speed, 3))round +
        #     " timeStamp " + str(time.monotonic()) )
        return self.speed                                                   # returns pdc in radians/second

    def setAngularVelocity(self, angularVelocity):
        self.targetSpeed = angularVelocity
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

        duty = sorted([-1, duty, 1])[1]                                     # place bounds on the motor commands
        self.motor.setDuty(round(duty, 2))                                  # must round to ensure driver handling!


if __name__ == "__main__":

    l_wheel = Wheel(1, 0x40, invert_encoder=True)                           # Left Motor  (ch1)
    r_wheel = Wheel(2, 0x41) 	                                            # Right Motor (ch2)

    while True:
        r_wheel.setAngularVelocity(np.pi)
        l_wheel.setAngularVelocity(np.pi)

        print(l_wheel.getAngularVelocity(), ' rad/s\t', r_wheel.getAngularVelocity(), ' rad/s')
