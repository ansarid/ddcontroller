import time
import math
import numpy as np
from scuttlepy import gpio
from scuttlepy import wheels
from scuttlepy import mpu
from fastlogging import LogInit

import os
if os.path.exists("robotTest.log"):
    os.remove("robotTest.log")

logger = LogInit(pathName="./robotTest.log")                                # Set up logger
logger.debug("ColumnA ColumnB ColumnC ColumnD")                             # Make columns

class SCUTTLE:

    def __init__(self):

        self.speed = 0
        self.heading = 0
        self.angularVelocity = 0
        self.globalPosition = np.array([0, 0])
        self.angularDisplacement = 0                                        # For tracking displacement between waypoints
        self.forwardDisplacement = 0                                        # For tracking displacement between waypoints

        # Robot Properties (SCUTTLE)

        self.wheelBase = 0.180                                              # L - meters    Measured from center of wheel base to inside edge of wheel.
        self.wheelRadius = 0.041                                            # R - meters
        self.wheelIncrements = np.array([0, 0])                             # Latest increments of wheels
        self.wheelSpeeds = [0 ,0]                                           # [Left wheel speed, Right wheel speed.]
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
        # self.l_encoderAddress = 0x40                                      # Left wheel encoder address
        # self.r_encoderAddress = 0x41                                      # Right wheel encoder address
        self.l_encoderAddress = 0x43
        self.r_encoderAddress = 0x40
        self.r_wheel = wheels.Wheel(self.r_motorChannel,                    # Create right wheel object
                                    self.r_encoderAddress
                                    )

        self.l_wheel = wheels.Wheel(self.l_motorChannel,                    # Create left wheel object
                                    self.l_encoderAddress,
                                    invert_encoder=True
                                    )

        self.imu = mpu.MPU()

    def setGlobal(self, pos):

        self.globalPosition = pos

    def setHeading(self, heading):

        self.heading = heading

    def getWheelIncrements(self):                                           # Get the wheel increment in radians

        self.l_wheel.positionInitial = self.l_wheel.positionFinal           # Transfer previous reading.
        self.r_wheel.positionInitial = self.r_wheel.positionFinal           # Transfer previous reading.
        self.timeInitial = self.timeFinal

        self.l_wheel.positionFinal = self.l_wheel.encoder.readPos()         # Reading, raw.
        self.r_wheel.positionFinal = self.r_wheel.encoder.readPos()         # Reading, raw.
        self.timeFinal = time.monotonic()

        wheelIncrements = np.array([self.l_wheel.getTravel(self.l_wheel.positionInitial,
                                                           self.l_wheel.positionFinal),
                                    self.r_wheel.getTravel(self.r_wheel.positionInitial,
                                                           self.r_wheel.positionFinal)])        # Store wheels travel in radians
        timeIncrement = self.timeFinal - self.timeInitial

        self.wheelSpeeds = wheelIncrements / timeIncrement
        self.l_wheel.speed = self.wheelSpeeds[0]                            # Overwrite attribute of the actual wheel object
        self.r_wheel.speed = self.wheelSpeeds[1]


        logger.debug("Time_Increment(s) " + str(round(timeIncrement, 3)) )
        logger.debug("Wheel_Increments(rad) " + str(round(wheelIncrements[0], 4))
                     + " " + str(round(wheelIncrements[1], 4)))

        return wheelIncrements

    def getChassis(self, displacement):                                     # This function returns the chassis displacement

        A = np.array([[          self.R/2,         self.R/2],
                      [-self.R/(2*self.L), self.R/(2*self.L)]])             # This matrix relates [PDL, PDR] to [XD,TD]
        B = displacement                                                    # This array should store phi displacements (in radians)
        C = np.matmul(A, B)                                                 # Perform matrix multiplication
        C = np.round(C, decimals=3)                                         # Round the matrix

        return C                                                            # Returns a matrix containing [dx (m), dTheta (rad)]

    def getChassisVelocity(self):                                           # Forward Kinematics
                                                                            # Function to update and return [x_dot,theta_dot]
        B = np.array([self.l_wheel.speed,                                   # Make an array of wheel speeds (rad/s)
                      self.r_wheel.speed])

        C = self.getChassis(B)                                              # Perform matrix multiplication
        self.speed = C[0]                                                   # Update speed of SCUTTLE [m/s]
        self.angularVelocity = C[1]                                         # Update angularVelocity = [rad/s]

        return [self.speed, self.angularVelocity]                           # Return [speed, angularVelocity]

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
                                                                            # Argument: [x_dot, theta_dot]
        C = self.getWheels(targetMotion)                                    # Perform matrix multiplication

        self.l_wheel.setAngularVelocity(C[0])                               # Set angularVelocity = [rad/s]
        self.r_wheel.setAngularVelocity(C[1])                               # Set angularVelocity = [rad/s]

        logger.debug("Wheel_Speeds(rad/s) " +                               # Report wheel speeds
             str(round(self.wheelSpeeds[0], 4)) + " " +
             str(round(self.wheelSpeeds[1], 4)))
        logger.debug("PhiSetPoints(rad/s) " +                               # PID controller target (should match PhiTarget)
            str(round(self.l_wheel.pid.SetPoint,3) ) + " " +
            str(round(self.r_wheel.pid.SetPoint,3))  )
        logger.debug("EffortLeft(duty) " +
            str(round(self.l_wheel.pid.PTerm,3)) + " " +
            str(round(self.l_wheel.pid.ITerm,3))     )
        logger.debug("EffortRight(duty) " +
            str(round(self.r_wheel.pid.PTerm,3)) + " " +
            str(round(self.r_wheel.pid.ITerm,3))     )

    def displacement(self):

        chassisIncrement = self.getChassis(self.getWheelIncrements())       # Get latest chassis travel (m, rad)
        self.forwardDisplacement = chassisIncrement[0]                      # Add the latest advancement(m) to the total
        self.angularDisplacement = chassisIncrement[1]                      # Add the latest advancement(rad) to the total

        logger.debug("Chassis_Increment(m,rad) " +
            str(round(chassisIncrement[0], 4)) + " " +
            str(round(chassisIncrement[1], 4)) + " " +
            str(time.monotonic()))

        self.loopStart = time.monotonic()                                   # Use for measuring loop time
        logger.debug("TimeStamp(s) " + str(self.loopStart))

        logger.debug("Gyro_raw(deg/s) " +
            str(round(self.imu.readAll()['gyro'][2], 3)) + " " +
            str(time.monotonic()))

    def stackDisplacement(self):                                            # Add the latest displacement to the global position
        theta = self.heading + ( self.angularDisplacement / 2 )             # Use the "halfway" vector as the stackup heading
        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, -s), (s, c)))                                     # Create the rotation matrix
        localVector = np.array([self.forwardDisplacement, 0])               # x value is increment and y value is always 0
        globalVector = np.matmul(R, localVector)
        self.globalPosition = self.globalPosition + globalVector            # Add the increment to the global position
        logger.debug("global_x(m) " +
            str(round(self.globalPosition[0], 3)) + " global_y(m) " +
            str(round(self.globalPosition[1], 3) ) )

    def drawVector(self):                                                   # Argument is an np array
        vector = self.point - self.globalPosition                           # The vector describing the next step
        self.vectorLength = math.sqrt(vector[0]**2 + vector[1]**2)          # Length in m
        self.vectorDirection = math.atan2(vector[1], vector[0])             # Calculate vector direction
        logger.debug("vectorLength(m) " +
            str(round(self.vectorLength, 3)) +
            " vectorDirection(deg) " +
            str(round(math.degrees(self.vectorDirection), 1)))

    def trajectory(self):
        span = math.radians(5)                                              # Span is the turning tolerance
        gap = self.vectorDirection - self.heading
        if gap > math.pi:                                                   # Large turns should be reversed
            gap += (2 * math.pi)
        if gap < -math.pi:
            gap -= (2 * math.pi)
        if gap > span:
            self.flip = 1                                                   # Positive turn needed
        elif gap < -span:
            self.flip = -1                                                  # Negative turn needed
        else:
            self.flip = 0                                                   # Go straight
        logger.debug("CurveFlip " + str(self.flip) )
        return self.flip

    def stackHeading(self):                                                 # Increment heading & ensure heading doesn't exceed 180
        self.heading = self.heading + self.angularDisplacement              # Update heading by the turn amount executed
        if self.heading > math.pi:
            self.heading += (2 * math.pi)
        if self.heading < -math.pi:
            self.heading += (2 * math.pi)
        logger.debug("heading(deg) " + str(round(math.degrees(self.heading), 3)))

    def checkLoop(self):
        self.loopFinish = time.monotonic()
        self.sleepTime = self.loopPeriod - (self.loopFinish - self.loopStart)
        logger.debug("sleepTime(s) " + str(round(self.sleepTime, 3)) )
        return(self.sleepTime)

    def setup(self):                                                        # Call this before moving to points

        self.getWheelIncrements()                                           # Get the very first nonzero readings fron enconders
        self.setMotion([0,0])                                               # Set speed zero
        self.displacement()                                                 # Increment the displacements (update robot attributes)
        self.stackDisplacement()                                            # Add the new displacement to global position
        self.stackHeading()                                                 # Add up the new heading

    def move(self, point):
        self.point = np.array(point)                                        # Set the destination point
        self.drawVector()                                                   # Draw vector to the destination
        self.trajectory()                                                   # Compute if turning is needed, populate "flip"

        if self.flip != 0:
            logger.debug("START_CURVING " + str(time.monotonic()))          # Log beginning of curve

        while abs(self.flip):                                               # Flip is +/-1 for turning.  flip is zero when heading points to target
            self.setMotion([self.cruiseRateTurning,
                            self.curveRate * self.flip])                    # Closed loop command for turning
            self.displacement()                                             # Increment the displacements (update robot attributes)
            self.stackDisplacement()                                        # Add the new displacement to global position
            self.stackHeading()                                             # Add up the new heading
            self.drawVector()                                               # Draw vector to the destination
            self.trajectory()                                               # Recompute if turning is needed
            self.checkLoop()                                                # Calculate how much to sleep
            if self.sleepTime > 0.001:
                time.sleep(self.sleepTime)

        if self.flip == 0:
            logger.debug("START_DRIVING " + str(time.monotonic()))          # Log beginning of straightaway

        while self.vectorLength > ( self.tolerance ):                       # Criteria to stop driving
            self.setMotion([self.cruiseRate, 0])                            # Closed loop driving forward
            self.displacement()                                             # Update the displacements
            self.stackDisplacement()
            self.stackHeading()
            self.drawVector()
            self.trajectory()
            self.checkLoop()                                                # Calculate how much to sleep
            if self.sleepTime > 0.001:
                time.sleep(self.sleepTime)

        logger.debug("SETTLE " + str(time.monotonic()))

        while self.speed != 0 and self.angularVelocity != 0:
            self.setMotion([0, 0])
            print( self.speed,  self.angularVelocity)
            time.sleep(0.035)

        logger.debug("Log_completed " + str(time.monotonic()))
        print("Movements finished. Log file: robotTest.log")