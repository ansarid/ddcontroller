#!/usr/bin/python3

import time
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

        self.wheelBase = WHEEL_BASE                             # L - meters    Measured from center of wheel base to inside edge of wheel.
        self.wheelRadius = WHEEL_RADIUS                         # R - meters
        self.wheelSpeeds = [0, 0]                               # [Left wheel speed, Right wheel speed.]

        self.leftEncoderAddress = LEFT_ENCODER_ADDRESS          # Left wheel encoder address
        self.rightEncoderAddress = RIGHT_ENCODER_ADDRESS        # Right wheel encoder address

        self.targetMotion = [0,0]

        self._loopStart = time.monotonic_ns()                   # Updates when we grab chassis displacements
        self._timeInitial = time.monotonic_ns()
        self._timeFinal = 0
        self._angularDisplacement = 0                           # For tracking displacement between waypoints
        self._forwardDisplacement = 0                           # For tracking displacement between waypoints
        self._wheelIncrements = np.array([0, 0])                # Latest increments of wheels

        self.leftMotorChannel = LEFT_MOTOR_CHANNEL 	            # Create Left Motor Object (pwm, digital)
        self.rightMotorChannel = RIGHT_MOTOR_CHANNEL 	        # Create Right Motor Object (pwm, digital)

        self.rightWheel = wheels.Wheel(self.rightMotorChannel,  # Create right wheel object
                                    self.rightEncoderAddress,
                                    openLoop=openLoop,
                                    )

        self.leftWheel = wheels.Wheel(self.leftMotorChannel,    # Create left wheel object
                                    self.leftEncoderAddress,
                                    invertEncoder=True,
                                    invertMotor=True,
                                    openLoop=openLoop,
                                    )

        self.stopped = False

        self._loopFreq = 50                                     # Target Wheel Loop frequency (Hz)
        self._wait = 1/self._loopFreq                           # Corrected wait time between encoder measurements (s)

        # self._loopTime = self._wait
        # self._startTime = time.monotonic_ns()

        self.wheelsThread = threading.Thread(target=self._wheelsLoop)
        self.wheelsThread.start()

    def _wheelsLoop(self):

        while not self.stopped:

            startTime = time.monotonic_ns()

            self.leftWheel.update()
            self.rightWheel.update()

            self.velocity, self.angularVelocity = self.getMotion()





            time.sleep(sorted([self._wait-((time.monotonic_ns()-startTime)/1e9), 0])[1])
            # print((time.monotonic_ns()-startTime)/1e6)                                    # Print loop time in ms

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

    def setMotion(self, targetMotion):                          # Take chassis speed and command wheels
                                                                # argument: [x_dot, theta_dot]
        # self.targetMotion = targetMotion

        L = self.wheelBase
        R = self.wheelRadius

        A = np.array([[ 1/R, -L/R],                             # This matrix relates chassis to wheels
                      [ 1/R,  L/R]])

        B = np.array([targetMotion[0],                          # Create an array for chassis speed
                      targetMotion[1]])

        C = np.matmul(A, B)                                     # Perform matrix multiplication

        self.leftWheel.setAngularVelocity(C[0])                 # Set angularVelocity = [rad/s]
        self.rightWheel.setAngularVelocity(C[1])                # Set angularVelocity = [rad/s]

    def getMotion(self):                                        # Forward Kinematics
                                                                # Function to update and return [x_dot,theta_dot]
        L = self.wheelBase
        R = self.wheelRadius

        A = np.array([[     R/2,     R/2],
                      [-R/(2*L), R/(2*L)]])                     # This matrix relates [PDL, PDR] to [XD,TD]

        B = np.array([self.leftWheel.getAngularVelocity(),      # make an array of wheel speeds (rad/s)
                      self.rightWheel.getAngularVelocity()])

        C = np.matmul(A, B)                                     # perform matrix multiplication
        C = np.round(C, decimals=3)                             # round the matrix

        self.velocity = C[0]                                  # Update speed of SCUTTLE [m/s]
        self.angularVelocity = C[1]                           # Update angularVelocity = [rad/s]

        return [self.velocity, self.angularVelocity]          # return [speed, angularVelocity]














if __name__ == "__main__":

    scuttle = SCUTTLE(openLoop=True)

    try:

        scuttle.setMotion([0.4, 0])
        # while scuttle.getGlobalPosition()[0] < 0.3:
        while True:
            # print(scuttle.velocity, scuttle.angularVelocity)
            # pos = scuttle.getGlobalPosition()
            # print(pos[0], pos[1], '\t', np.degrees(scuttle.getHeading()))
            time.sleep(1/45)

        scuttle.setMotion([0, 0])
        # print(pos[0], pos[1], '\t', np.degrees(scuttle.getHeading()))

    except KeyboardInterrupt:

        pass

    finally:

        scuttle.stop()
