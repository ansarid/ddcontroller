#!/usr/bin/python3

import time
import math
import threading
import numpy as np
import scuttlepy.wheels as wheels
from scuttlepy.constants import *

# from fastlogging import LogInit

class SCUTTLE:

    def __init__(self, openLoop=False,):

        self.heading = 0
        self.velocity = 0
        self.angularVelocity = 0
        self.globalPosition = np.array([0, 0])

        self.wheelBase = WHEEL_BASE                                 # L - meters    Measured from center of wheel base to inside edge of wheel.
        self.wheelRadius = WHEEL_RADIUS                             # R - meters
        self.wheelSpeeds = [0, 0]                                   # [Left wheel speed, Right wheel speed.]

        self.leftEncoderAddress = LEFT_ENCODER_ADDRESS              # Left wheel encoder address
        self.rightEncoderAddress = RIGHT_ENCODER_ADDRESS            # Right wheel encoder address

        self.targetMotion = [0,0]

        self._loopStart = time.monotonic_ns()                          # Updates when we grab chassis displacements
        self._timeInitial = time.monotonic_ns()
        self._timeFinal = 0
        self._angularDisplacement = 0                               # For tracking displacement between waypoints
        self._forwardDisplacement = 0                               # For tracking displacement between waypoints
        self._wheelIncrements = np.array([0, 0])                    # Latest increments of wheels

        self.leftMotorChannel = LEFT_MOTOR_CHANNEL 	                # Create Left Motor Object (pwm, digital)
        self.rightMotorChannel = RIGHT_MOTOR_CHANNEL 	            # Create Right Motor Object (pwm, digital)

        self.rightWheel = wheels.Wheel(self.rightMotorChannel,      # Create right wheel object
                                    self.rightEncoderAddress,
                                    openLoop=openLoop,
                                    )

        self.leftWheel = wheels.Wheel(self.leftMotorChannel,        # Create left wheel object
                                    self.leftEncoderAddress,
                                    openLoop=openLoop,
                                    invert_encoder=True,
                                    )

        self.stopped = False

        self._loopFreq = 50                                         # Target Wheel Loop frequency (Hz)
        self._wait = 1/self._loopFreq                               # Corrected wait time between encoder measurements (s)

        self._loopTime = self._wait
        self._startTime = time.monotonic_ns()

        self.wheelsThread = threading.Thread(target=self._wheelsLoop)
        self.wheelsThread.start()

    def _wheelsLoop(self):

        while not self.stopped:

            self._startTime = time.monotonic_ns()                       # reset startTime
            # self.setMotion(self.targetMotion)                           # Set target velocity

            leftWheelDistance = self.leftWheel.getTravel()
            rightWheelDistance = self.rightWheel.getTravel()

            if leftWheelDistance >= rightWheelDistance:

                distanceTraveled = (rightWheelDistance+leftWheelDistance)/2
                self.heading = self.heading + ((leftWheelDistance - rightWheelDistance)/self.wheelBase)

            elif leftWheelDistance <= rightWheelDistance:

                distanceTraveled = (leftWheelDistance + rightWheelDistance)/2
                self.heading = self.heading + ((rightWheelDistance - leftWheelDistance)/self.wheelBase)

            self.globalPosition = [self.globalPosition[0] + (distanceTraveled*np.cos(self.heading)), self.globalPosition[1] + (distanceTraveled*np.sin(self.heading))]

        self.rightWheel.stop()
        self.leftWheel.stop()

    def stop(self):
        self.setMotion([0, 0])
        self.stopped = True
        self.wheelsThread.join()

    def setGlobalPosition(self, pos):
        self.globalPosition = pos
        return self.globalPosition

    def setHeading(self, heading):
        self.heading = heading
        return self.heading

    def getGlobalPosition(self):
        return self.globalPosition

    def getHeading(self):
        return self.heading

    def calculateWheelSpeeds(self, targetMotion):                           # Inverse Kinematic function. Take x_dot, theta_dot as arguments

        L = self.wheelBase
        R = self.wheelRadius

        A = np.array([[ 1/R, -L/R],                                         # This matrix relates chassis to wheels
                      [ 1/R,  L/R]])

        B = np.array([targetMotion[0],                                      # Create an array for chassis speed
                      targetMotion[1]])

        C = np.matmul(A, B)                                                 # Perform matrix multiplication
        return C                                                            # Returns Phi_dots, (rad or rad/s)

    def setMotion(self, targetMotion):                                      # Take chassis speed and command wheels
                                                                            # argument: [x_dot, theta_dot]
        C = self.calculateWheelSpeeds(targetMotion)

        self.leftWheel.setAngularVelocity(C[0])                             # Set angularVelocity = [rad/s]
        self.rightWheel.setAngularVelocity(C[1])                            # Set angularVelocity = [rad/s]
