#!/usr/bin/python3

import time
import math
import threading
import numpy as np
import scuttlepy.wheels as wheels
# from fastlogging import LogInit

from adafruit_platformdetect import Detector
detector = Detector()

class SCUTTLE:

    def __init__(self, openLoop=False,):

        self.heading = 0
        self.velocity = 0
        self.angularVelocity = 0
        self.globalPosition = np.array([0, 0])

        self.wheelBase = 0.1775                                     # L - meters    Measured from center of wheel base to inside edge of wheel.
        self.wheelRadius = 0.04165                                  # R - meters
        self.wheelSpeeds = [0, 0]                                   # [Left wheel speed, Right wheel speed.]

        self.leftEncoderAddress = 0x43                                # Left wheel encoder address
        self.rightEncoderAddress = 0x41                                # Right wheel encoder address

        self.targetMotion = [0,0]

        self._loopStart = time.monotonic()                          # Updates when we grab chassis displacements
        self._timeInitial = time.monotonic()
        self._timeFinal = 0
        self._angularDisplacement = 0                               # For tracking displacement between waypoints
        self._forwardDisplacement = 0                               # For tracking displacement between waypoints
        self._wheelIncrements = np.array([0, 0])                    # Latest increments of wheels

        if detector.board.BEAGLEBONE_BLUE:

            self.leftMotorChannel = 1
            self.rightMotorChannel = 2

        elif detector.board.any_raspberry_pi_40_pin:

            self.leftMotorChannel = (15,16) 	                        # Create Left Motor Object (pwm, digital)
            self.rightMotorChannel = (11,12) 	                        # Create Right Motor Object (pwm, digital)

        elif detector.board.JETSON_NANO:

            self.leftMotorChannel = (32,29)                           # Create Left Motor Object (pwm, digital)
            self.rightMotorChannel = (33,31)                           # Create Right Motor Object (pwm, digital)

        self.rightWheel = wheels.Wheel(self.rightMotorChannel,            # Create right wheel object
                                    self.rightEncoderAddress,
                                    openLoop=openLoop,
                                    )

        self.leftWheel = wheels.Wheel(self.leftMotorChannel,            # Create left wheel object
                                    self.leftEncoderAddress,
                                    openLoop=openLoop,
                                    invert_encoder=True,
                                    )

        self.stopped = False

        self._loopFreq = 50                                         # Target Wheel Loop frequency (Hz)
        self._wait = 1/self._loopFreq                               # Corrected wait time between encoder measurements (s)

        self._loopTime = self._wait
        self._startTime = time.monotonic()

        self.wheelsThread = threading.Thread(target=self._wheelsLoop)
        self.wheelsThread.start()

    def _wheelsLoop(self):

        while not self.stopped:

            self.setMotion(self.targetMotion)                       # Set target velocity

            displacement = self.getDisplacement()
            self.stackDisplacement(displacement[0], displacement[1])
            self.stackHeading(displacement[1])

            self._loopTime = (time.monotonic()-self._startTime)     # Calculate loop time
            _loopTimeOffset = (1/self._loopFreq)-self._loopTime     # Calculate time difference between target and actual loop time
            self._wait += _loopTimeOffset                           # Adjust wait time to achieve target
            self._startTime = time.monotonic()                      # reset startTime

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

    def _getWheelIncrements(self):                                           # get the wheel increment in radians

        self.leftWheel.positionInitial = self.leftWheel._positionFinal       # transfer previous reading.
        self.rightWheel.positionInitial = self.rightWheel._positionFinal     # transfer previous reading.
        self._timeInitial = self._timeFinal

        self.leftWheel._positionFinal = self.leftWheel.encoder.readPos()     # reading, raw.
        self.rightWheel._positionFinal = self.rightWheel.encoder.readPos()   # reading, raw.
        self._timeFinal = time.monotonic()

        wheelIncrements = np.array([self.leftWheel.getRotation(self.leftWheel.positionInitial,
                                                           self.leftWheel._positionFinal),
                                    self.rightWheel.getRotation(self.rightWheel.positionInitial,
                                                           self.rightWheel._positionFinal)])        # store wheels travel in radians
        timeIncrement = self._timeFinal - self._timeInitial

        self.wheelSpeeds = wheelIncrements / timeIncrement                  # speed = distance/time
        return wheelIncrements

    def getChassis(self, displacement):                                     # this function returns the chassis displacement

        L = self.wheelBase
        R = self.wheelRadius

        A = np.array([[     R/2,     R/2],
                      [-R/(2*L), R/(2*L)]])                                 # This matrix relates [PDL, PDR] to [XD,TD]

        B = displacement                                                    # this array should store phi displacements (in radians)
        C = np.matmul(A, B)                                                 # perform matrix multiplication
        C = np.round(C, decimals=3)                                         # round the matrix

        return C                                                            # returns a matrix containing [dx (m), dTheta (rad)]

    def getMotion(self):                                           # Forward Kinematics
                                                                            # Function to update and return [x_dot,theta_dot]
        B = np.array([self.leftWheel.getAngularVelocity(),                                   # make an array of wheel speeds (rad/s)
                      self.rightWheel.getAngularVelocity()])

        C = self.getChassis(B)                                              # Perform matrix multiplication
        self.velocity = C[0]                                                # Update speed of SCUTTLE [m/s]
        self.angularVelocity = C[1]                                         # Update angularVelocity = [rad/s]

        return [self.velocity, self.angularVelocity]                        # return [speed, angularVelocity]

    def calculateWheelSpeeds(self, targetMotion):                                     # Inverse Kinematic function. Take x_dot, theta_dot as arguments

        L = self.wheelBase
        R = self.wheelRadius

        A = np.array([[ 1/R, -L/R],                                         # This matrix relates chassis to wheels
                      [ 1/R,  L/R]])

        B = np.array([targetMotion[0],                                     # Create an array for chassis speed
                      targetMotion[1]])

        C = np.matmul(A, B)                                                 # Perform matrix multiplication
        return C                                                            # Returns Phi_dots, (rad or rad/s)

    def setMotion(self, targetMotion):                                      # Take chassis speed and command wheels
                                                                            # argument: [x_dot, theta_dot]
        self.targetMotion = targetMotion
        C = self.calculateWheelSpeeds(targetMotion)

        self.leftWheel.setAngularVelocity(C[0])                             # Set angularVelocity = [rad/s]
        self.rightWheel.setAngularVelocity(C[1])                            # Set angularVelocity = [rad/s]

    def getDisplacement(self):                               # store dispalcement info to the log var_AngularAndForwardDisplacements

        chassisIncrement = self.getChassis(self._getWheelIncrements())       # get latest chassis travel (m, rad)
        self._forwardDisplacement = chassisIncrement[0]                     # add the latest advancement(m) to the total
        self._angularDisplacement = chassisIncrement[1]                     # add the latest advancement(rad) to the total

        self._loopStart = time.monotonic()                                  # use for measuring loop time

        return np.array([self._forwardDisplacement, self._angularDisplacement])

                                                                            # THIS MAY BE THE TRICKIEST FUNCTION OF THE WHOLE PROGRAM.  IT IS SPECIAL BECAUSE IT USES THE
                                                                            # INFORMATION ABOUT THE LATEST ANGULAR DISPLACEMENT TO CALCULATE A FORWARD DISPLACEMENT
                                                                            # IN CARTESIAN SYSTEM.

    def stackDisplacement(self, forwardDisplacement, angularDisplacement):  # add the latest displacement to the global position var_AngularAndForwardDisplacements
        theta = self.heading + ( angularDisplacement / 2 )                  # use the "halfway" vector as the stackup heading
        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, -s), (s, c)))                                     # create the rotation matrix
        localVector = np.array([forwardDisplacement, 0])                    # x value is increment and y value is always 0
        globalVector = np.matmul(R, localVector)
        self.globalPosition = self.globalPosition + globalVector            # add the increment to the global position

        return self.globalPosition

    def stackHeading(self, angularDisplacement):                            # increment heading & ensure heading doesn't exceed 180 var_chassisImmediateAngularDisplacement
        self.heading += angularDisplacement                                 # update heading by the turn amount executed
        if self.heading > math.pi:
            self.heading += (2 * math.pi)
        if self.heading < -math.pi:
            self.heading += (2 * math.pi)

        return self.heading
