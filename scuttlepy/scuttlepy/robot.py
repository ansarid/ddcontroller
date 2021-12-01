#!/usr/bin/python3

import time
import math
import logging
import threading
import numpy as np
from scuttlepy import wheels
from scuttlepy import mpu
# from fastlogging import LogInit

from adafruit_platformdetect import Detector
detector = Detector()

class SCUTTLE:

    def __init__(self, configFile='~/.config/scuttle/config.yaml', debugging=False, debugFile=None, openLoop=False,):

        self.debugging = debugging
        self.debugFile = debugFile

        if self.debugging:
            if self.debugFile:
                logging.basicConfig(filename=self.debugFile, format='%(asctime)s %(message)s', filemode='w')
                self.logger = logging.getself.logger()                                                # create an object
                self.logger.setLevel(logging.DEBUG)                                              # set threshold of self.logger to DEBUG
                self.logger.disabled = False
            else:
                print('Please specify a debug file name!')
                self.debugging = False

        self.speed = 0
        self.heading = 0
        self.angularVelocity = 0
        self.globalPosition = np.array([0, 0])
        self.angularDisplacement = 0                                        # For tracking displacement between waypoints
        self.forwardDisplacement = 0                                        # For tracking displacement between waypoints

        self.wheelBase = 0.180                                              # L - meters    Measured from center of wheel base to inside edge of wheel.
        self.wheelRadius = 0.041                                            # R - meters
        self.wheelIncrements = np.array([0, 0])                             # Latest increments of wheels
        self.wheelSpeeds = [0, 0]                                           # [Left wheel speed, Right wheel speed.]
        self.timeInitial = time.monotonic()
        self.timeFinal = 0
        self.loopPeriod = 0.060                                             # How fast to make the loop (s)
        self.loopStart = time.monotonic()                                   # Updates when we grab chassis displacements
        self.sleepTime = 0                                                  # Time to sleep updated per loop

        self.cruiseRate = 0.2                                               # Fwd driving speed, m/s
        self.cruiseRateTurning = 0.02                                       # Reduced fwd speed during turning
        self.curveRadius = 0.020                                            # Curve radius (m)
        self.curveRate = (self.cruiseRateTurning / self.curveRadius)        # Curve rotational speed, thetadot (rad/s)
        self.tolerance = 0.100                                              # 25mm for first test
        self.flip = 0                                                       # Go straight
        self.vectorLength = 0

        self.l_motorChannel = 1
        self.r_motorChannel = 2
        self.l_encoderAddress = 0x40                                        # Left wheel encoder address
        self.r_encoderAddress = 0x41                                        # Right wheel encoder address

        self.targetMotion = [0,0]

        if detector.board.BEAGLEBONE_BLUE:

            self.l_motorChannel = 1
            self.r_motorChannel = 2

        elif detector.board.RASPBERRY_PI_40_PIN or detector.board.JETSON_NANO:

            self.l_motorChannel = (32,29)
            self.r_motorChannel = (33,31)

        self.r_wheel = wheels.Wheel(self.r_motorChannel,                    # Create right wheel object
                                    self.r_encoderAddress,
                                    openLoop=openLoop,
                                    )

        self.l_wheel = wheels.Wheel(self.l_motorChannel,                    # Create left wheel object
                                    self.l_encoderAddress,
                                    openLoop=openLoop,
                                    invert_encoder=True,
                                    )

        self.stopped = False

        self.setup()                                                        # Take the first encoder readings, establish start
        # self.imu = mpu.IMU()
        self.loopFreq = 50                                                  # Target Wheel Loop frequency (Hz)
        self.wait = 1/self.loopFreq                                         # Corrected wait time between encoder measurements (s)

        self.loopTime = self.wait
        self.startTime = time.monotonic()

        self.wheelsThread = threading.Thread(target=self._wheelsLoop)
        self.wheelsThread.start()

    def _wheelsLoop(self):

        while not self.stopped:

            self.setMotion(self.targetMotion)                       # Set target velocity

            self.loopTime = (time.monotonic()-self.startTime)               # Calculate loop time
            loopTimeOffset = (1/self.loopFreq)-self.loopTime                # Calculate time difference between target and actaul loop time
            self.wait += loopTimeOffset                                     # Adjust wait time to achieve target
            self.startTime = time.monotonic()                               # reset startTime

        self.r_wheel.stop()
        self.l_wheel.stop()

    def stop(self):
        self.setMotion([0, 0])
        self.stopped = True
        self.wheelsThread.join()

    def setGlobal(self, pos):

        self.globalPosition = pos

    def setHeading(self, heading):

        self.heading = heading

    def getWheelIncrements(self):                                           # get the wheel increment in radians var_bothWheelEncoderPositions

        self.l_wheel.positionInitial = self.l_wheel.positionFinal           # transfer previous reading.
        self.r_wheel.positionInitial = self.r_wheel.positionFinal           # transfer previous reading.
        self.timeInitial = self.timeFinal

        self.l_wheel.positionFinal = self.l_wheel.encoder.readPos()         # reading, raw.
        self.r_wheel.positionFinal = self.r_wheel.encoder.readPos()         # reading, raw.
        self.timeFinal = time.monotonic()

        wheelIncrements = np.array([self.l_wheel.getTravel(self.l_wheel.positionInitial,
                                                           self.l_wheel.positionFinal),
                                    self.r_wheel.getTravel(self.r_wheel.positionInitial,
                                                           self.r_wheel.positionFinal)])        # store wheels travel in radians
        timeIncrement = self.timeFinal - self.timeInitial

        self.wheelSpeeds = wheelIncrements / timeIncrement                  # speed = distance/time

        if self.debugging:

            self.logger.debug("Time_Increment(s) " + str(round(timeIncrement, 3)) )
            self.logger.debug("Wheel_Increments(rad) " + str(round(wheelIncrements[0], 4))
                        + " " + str(round(wheelIncrements[1], 4)))

        return wheelIncrements

    def getChassis(self, displacement):                                     # this function returns the chassis displacement var_ImmediateWheelsDisplacementOrPhis

        L = self.wheelBase
        R = self.wheelRadius

        A = np.array([[     R/2,     R/2],
                      [-R/(2*L), R/(2*L)]])                                 # This matrix relates [PDL, PDR] to [XD,TD]

        B = displacement                                                    # this array should store phi displacements (in radians)
        C = np.matmul(A, B)                                                 # perform matrix multiplication
        C = np.round(C, decimals=3)                                         # round the matrix

        return C                                                            # returns a matrix containing [dx (m), dTheta (rad)]

    def getChassisVelocity(self):                                           # Forward Kinematics
                                                                            # Function to update and return [x_dot,theta_dot]
        B = np.array([self.l_wheel.speed,                                   # make an array of wheel speeds (rad/s)
                      self.r_wheel.speed])

        C = self.getChassis(B)                                              # Perform matrix multiplication
        self.speed = C[0]                                                   # Update speed of SCUTTLE [m/s]
        self.angularVelocity = C[1]                                         # Update angularVelocity = [rad/s]

        return [self.speed, self.angularVelocity]                           # return [speed, angularVelocity]

    def getWheels(self, chassisValues):                                     # Inverse Kinematic function. Take x_dot, theta_dot as arguments

        L = self.wheelBase
        R = self.wheelRadius

        A = np.array([[ 1/R, -L/R],                                         # This matrix relates chassis to wheels
                      [ 1/R,  L/R]])

        B = np.array([chassisValues[0],                                     # Create an array for chassis speed
                      chassisValues[1]])

        C = np.matmul(A, B)                                                 # Perform matrix multiplication

        return C                                                            # Returns Phi_dots, (rad or rad/s)

    def setMotion(self, targetMotion):                                      # Take chassis speed and command wheels
                                                                            # argument: [x_dot, theta_dot]
        self.targetMotion = targetMotion
        C = self.getWheels(targetMotion)                                    # Perform matrix multiplication

        self.l_wheel.setAngularVelocity(C[0])                               # Set angularVelocity = [rad/s]
        self.r_wheel.setAngularVelocity(C[1])                               # Set angularVelocity = [rad/s]

        if self.debugging:
            self.logger.debug("Wheel_Speeds(rad/s) " +                               # report wheel speeds
                str(round(self.wheelSpeeds[0], 4)) + " " +
                str(round(self.wheelSpeeds[1], 4)))
            self.logger.debug("PhiSetPoints(rad/s) " +                               # PID controller target (should match PhiTarget)
                str(round(self.l_wheel.pid.SetPoint,3) ) + " " +
                str(round(self.r_wheel.pid.SetPoint,3))  )
            self.logger.debug("EffortLeft(duty) " +
                str(round(self.l_wheel.pid.PTerm,3)) + " " +
                str(round(self.l_wheel.pid.ITerm,3))     )
            self.logger.debug("EffortRight(duty) " +
                str(round(self.r_wheel.pid.PTerm,3)) + " " +
                str(round(self.r_wheel.pid.ITerm,3))     )

    def displacement(self, chassisIncrement):                               # store dispalcement info to the log var_AngularAndForwardDisplacements

        # chassisIncrement = self.getChassis(self.getWheelIncrements())       # get latest chassis travel (m, rad)
        self.forwardDisplacement = chassisIncrement[0]                      # add the latest advancement(m) to the total
        self.angularDisplacement = chassisIncrement[1]                      # add the latest advancement(rad) to the total

        if self.debugging:
            self.logger.debug("Chassis_Increment(m,rad) " +
                str(round(chassisIncrement[0], 4)) + " " +
                str(round(chassisIncrement[1], 4)) + " " +
                str(time.monotonic()))

        self.loopStart = time.monotonic()                                   # use for measuring loop time

        if self.debugging:
            self.logger.debug("TimeStamp(s) " + str(self.loopStart))

            self.logger.debug("Gyro_raw(deg/s) " +
                str(round(self.imu.getHeading(), 3)) + " " +
                str(time.monotonic()))

        return np.array([self.forwardDisplacement, self.angularDisplacement])

                                                                            # THIS MAY BE THE TRICKIEST FUNCTION OF THE WHOLE PROGRAM.  IT IS SPECIAL BECAUSE IT USES THE
                                                                            # INFORMATION ABOUT THE LATEST ANGULAR DISPLACEMENT TO CALCULATE A FORWARD DISPLACEMENT
                                                                            # IN CARTESIAN SYSTEM.

    def stackDisplacement(self, forwardDisplacement, angularDisplacement):                                            # add the latest displacement to the global position var_AngularAndForwardDisplacements
        theta = self.heading + ( angularDisplacement / 2 )             # use the "halfway" vector as the stackup heading
        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, -s), (s, c)))                                     # create the rotation matrix
        localVector = np.array([forwardDisplacement, 0])               # x value is increment and y value is always 0
        globalVector = np.matmul(R, localVector)
        self.globalPosition = self.globalPosition + globalVector            # add the increment to the global position

        if self.debugging:
            self.logger.debug("global_x(m) " +
                str(round(self.globalPosition[0], 3)) + " global_y(m) " +
                str(round(self.globalPosition[1], 3) ) )

        return self.globalPosition

    def drawVector(self, globalPosition, point):                            # argument is an np array
        vector = point - globalPosition                                     # the vector describing the next step
        self.vectorLength = math.sqrt(vector[0]**2 + vector[1]**2)          # length in m
        self.vectorDirection = math.atan2(vector[1], vector[0])             # discover vector direction

        if self.debugging:
            self.logger.debug("vectorLength(m) " +
                str(round(self.vectorLength, 3)) +
                " vectorDirection(deg) " +
                str(round(math.degrees(self.vectorDirection), 1)))

        vector = np.array([self.vectorLength, self.vectorDirection])
        return vector

    def trajectory(self, vector):
        span = math.radians(5)                                              # span is the turning tolerance
        gap = self.vectorDirection - self.heading
        if gap > math.pi:                                                   # large turns should be reversed
            gap += (2 * math.pi)
        if gap < -math.pi:
            gap -= (2 * math.pi)
        if gap > span:
            self.flip = 1                                                   # positive turn needed
        elif gap < -span:
            self.flip = -1                                                  # negative turn needed
        else:
            self.flip = 0                                                   # go straight

        if self.debugging:
            self.logger.debug("CurveFlip " + str(self.flip) )

        return self.flip

    def stackHeading(self, angularDisplacement):                                                 # increment heading & ensure heading doesn't exceed 180 var_chassisImmediateAngularDisplacement
        self.heading += angularDisplacement              # update heading by the turn amount executed
        if self.heading > math.pi:
            self.heading += (2 * math.pi)
        if self.heading < -math.pi:
            self.heading += (2 * math.pi)

        if self.debugging:
            self.logger.debug("heading(deg) " + str(round(math.degrees(self.heading), 3)))

        return self.heading

    def checkLoop(self):
        self.loopFinish = time.monotonic()
        self.sleepTime = self.loopPeriod - (self.loopFinish - self.loopStart)

        if self.debugging:
            self.logger.debug("sleepTime(s) " + str(round(self.sleepTime, 3)))

        return self.sleepTime

    def setup(self):                                                        # call this before moving to points

        self.getWheelIncrements()                                           # get the very first nonzero readings fron encoders var_bothWheelEncoderPositions
        self.setMotion(self.targetMotion)                                               # set speed zero [no inputs needed]
        forwardDisplacement, angularDisplacement = self.displacement(self.getChassis(self.getWheelIncrements()))       # increment the displacements (update robot attributes) var_AngularAndForwardDisplacements
                                                                            # for getChassis: var_ImmediateWheelsDisplacementOrPhis
        self.stackDisplacement(forwardDisplacement, angularDisplacement)    # add the new displacement to global position var_AngularAndForwardDisplacements
        self.stackHeading(angularDisplacement)                                                 # add up the new heading  var_chassisImmediateAngularDisplacement

    def move(self, point):
        self.point = np.array(point)                                        # set the destination point  var_destinationPoint
        vector = self.drawVector(self.point, self.globalPosition)                    # draw vector to the destination var_currentLocationAndDestinationPoints
        self.trajectory(vector)                                                   # compute if turning is needed, populate "flip" 

        if self.debugging:
            if self.flip != 0:
                self.logger.debug("START_CURVING " + str(time.monotonic()))          # log beginning of curve

        while abs(self.flip):                                               # flip is +/-1 for turning.  flip is zero when heading points to target
            self.setMotion([self.cruiseRateTurning, self.curveRate * self.flip])   # closed loop command for turning
            forwardDisplacement, angularDisplacement = self.displacement(self.getChassis(self.getWheelIncrements()))       # increment the displacements (update robot attributes) var_AngularAndForwardDisplacements
            self.globalPosition = self.stackDisplacement(forwardDisplacement, angularDisplacement)
            self.heading = self.stackHeading(angularDisplacement)
            vector = self.drawVector(self.globalPosition, point)                                           # draw vector to the destination
            self.trajectory(vector)                                               # recompute if turning is needed
            self.checkLoop()                                                # calculate how much to sleep
            if self.sleepTime > 0.001:
                time.sleep(self.sleepTime)

        if self.debugging:
            if self.flip == 0:
                self.logger.debug("START_DRIVING " + str(time.monotonic()))          # log beginning of straightaway

        while self.vectorLength > ( self.tolerance ):                       # criteria to stop driving
            self.setMotion([self.cruiseRate, 0])                            # closed loop driving forward
            # self.displacement(self.getChassis(self.getWheelIncrements()))                                             # update the displacements
            forwardDisplacement, angularDisplacement = self.displacement(self.getChassis(self.getWheelIncrements()))       # increment the displacements (update robot attributes) var_AngularAndForwardDisplacements

            self.globalPosition = self.stackDisplacement(forwardDisplacement, angularDisplacement)
            self.heading = self.stackHeading(angularDisplacement)
            vector = self.drawVector(self.globalPosition, point)
            self.trajectory(vector)
            self.checkLoop()                                                # calculate how much to sleep
            if self.sleepTime > 0.001:
                time.sleep(self.sleepTime)

        if self.debugging:
            self.logger.debug("SETTLE " + str(time.monotonic()))

        while self.speed != 0 and self.angularVelocity != 0:
            self.setMotion([0, 0])
            time.sleep(0.035)

        if self.debugging:
            self.logger.debug("Log_completed " + str(time.monotonic()))

if __name__ == "__main__":

    scuttle = SCUTTLE()

    try:
        while True:
            scuttle.setMotion([0, 4])
            time.sleep(5)
            scuttle.setMotion([-2, 0])
            time.sleep(5)
            scuttle.setMotion([2, 0])
            time.sleep(5)

    except KeyboardInterrupt:
        scuttle.stop()
        pass
