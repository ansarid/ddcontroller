import time
import math
# import logging
import numpy as np
from scuttlepy import gpio
from scuttlepy import wheels
from scuttlepy import mpu
from fastlogging import LogInit

import os
if os.path.exists("robotTest.log"):
    os.remove("robotTest.log")

logger = LogInit(pathName="./robotTest.log")

# logger.debug("This is a debug message.")
# Create and configure logger
# logging.basicConfig(filename="robotTest.log", format='%(asctime)s %(message)s', filemode='w')
# logger = logging.getLogger()                                                # create an object
# logger.setLevel(logging.DEBUG)                                              # set threshold of logger to DEBUG
logger.debug("ColumnA ColumnB ColumnC ColumnD")

gpio.write(1, 3, 0) # port 1 pin 3 deactivate


class SCUTTLE:

    def __init__(self):

        self.speed = 0
        self.heading = 0
        self.compass = 0
        self.angularVelocity = 0
        self.turnRate = 0
        self.globalPosition = np.array([0, 0])
        self.angularDisplacement = 0                                        # for tracking displacement between waypoints
        self.forwardDisplacement = 0                                        # for tracking displacement between waypoints

        self.l_motorChannel = 1
        self.r_motorChannel = 2

        # self.l_encoderAddress = 0x40
        # self.r_encoderAddress = 0x41

        self.l_encoderAddress = 0x43
        self.r_encoderAddress = 0x40

        # self.wheelBase = 0.201                                              # L - meters
        self.wheelBase = 0.180                                              # L - meters
        # self.wheelBase = 0.170                                              # L - meters

        self.wheelRadius = 0.041                                            # R - meters
        self.wheelIncrements = np.array([0, 0])                             # latest increments of wheels
        self.wheelSpeeds = 0
        self.timeInitial = time.monotonic()
        self.timeFinal = 0
        self.loopPeriod = 0.060                                             # how fast to make the loop (s)
        self.loopStart = time.monotonic()                                   # updates when we grab chassis displacements
        self.sleeptime = 0                                                  # time to sleep updated per loop

        self.L = self.wheelBase
        self.R = self.wheelRadius

        self.rampDown = 0.020                                               # m
        self.overSteer = math.radians(10)                                   # deg

        self.cruiseRate = 0.240                                             # fwd driving speed, m/s
        self.curveRadius = 0.300                                            # curve radius (m)
        self.curveRate = self.cruiseRate / self.curveRadius                 # curve rotational speed (rad/s)
        self.L2 = 0                                                         # amount to cut from straight path
        self.arcLen = 0
        self.tolerance = 0.100                                              # 25mm for first test
        self.flip = 0                                                       # go straight
        self.vectorLength = 0

        self.batteryVoltage = 0

        self.r_wheel = wheels.Wheel(self.r_motorChannel,
                                    self.r_encoderAddress)

        self.l_wheel = wheels.Wheel(self.l_motorChannel,
                                    self.l_encoderAddress,
                                    invert_encoder=True)

        self.imu = mpu.MPU()

    def setGlobal(self, pos):

        self.globalPosition = pos

    def setHeading(self, heading):

        self.heading = heading

    def getWheelIncrements(self):                                           # get the wheel increment in radians

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

        self.wheelSpeeds = wheelIncrements / timeIncrement
        self.l_wheel.speed = self.wheelSpeeds[0]                            # overwrite attribute of the actual wheel object
        self.r_wheel.speed = self.wheelSpeeds[1]


        logger.debug("Time_Increment(s) " + str(round(timeIncrement, 3)) )
        logger.debug("Wheel_Increments(rad) " + str(round(wheelIncrements[0], 4))
                     + " " + str(round(wheelIncrements[1], 4)))
        logger.debug("Wheel_Speeds(rad/s) " + str(round(self.wheelSpeeds[0], 4))
                     + " " + str(round(self.wheelSpeeds[1], 4)))
        logger.debug("PID_Speeds(rad/s) " + str(round(self.l_wheel.speed, 4))
                     + " " + str(round(self.r_wheel.speed, 4)))

        return wheelIncrements

    def getChassis(self, displacement):                                     # this function returns the chassis displacement

        A = np.array([[          self.R/2,         self.R/2],
                      [-self.R/(2*self.L), self.R/(2*self.L)]])             # This matrix relates [PDL, PDR] to [XD,TD]
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
        C = self.getWheels(targetMotion)                                    # Perform matrix multiplication

        logger.debug("PhiTargets(rad/s) " + str(round(C[0],3))  +          # indicate wheelspeed targets in log
            " " + str(round(C[1],3)))

        self.l_wheel.setAngularVelocity(C[0])                               # Set angularVelocity = [rad/s]
        self.r_wheel.setAngularVelocity(C[1])                               # Set angularVelocity = [rad/s]

        logger.debug("PhiSetPoints(rad/s) " +                               # PID controller target (should match PhiTarget)
            str(round(self.l_wheel.pid.SetPoint,3) ) + " " +
            str(round(self.r_wheel.pid.SetPoint,3))  )

    def displacement(self):

        chassisIncrement = self.getChassis(self.getWheelIncrements())       # get latest chassis travel (m, rad)
        self.forwardDisplacement = chassisIncrement[0]                      # add the latest advancement(m) to the total
        self.angularDisplacement = chassisIncrement[1]                      # add the latest advancement(rad) to the total

        logger.debug("Chassis_Increment(m,rad) " +
                    str(round(chassisIncrement[0], 4)) + " " +
                    str(round(chassisIncrement[1], 4)) + " " +
                    str(time.monotonic()))

        self.loopStart = time.monotonic()                                   # use for measuring loop timehttps://prod.liveshare.vsengsaas.visualstudio.com/join?60A021E3072343B1F9E3A422F65353A190FD
        logger.debug("TimeStamp(s) " + str(self.loopStart))

        logger.debug("Gyro_raw(deg/s) " +
            str(round(self.imu.readAll()['gyro'][2], 3)) + " " +
            str(time.monotonic()))

    def stackDisplacement(self):                                            # add the latest displacement to the global position
        theta = self.heading + ( self.angularDisplacement / 2 )             # use the "halfway" vector as the stackup heading
        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, -s), (s, c)))                                     # create the rotation matrix
        localVector = np.array([self.forwardDisplacement, 0])               # x value is increment and y value is always 0
        globalVector = np.matmul(R, localVector)
        self.globalPosition = self.globalPosition + globalVector            # add the increment to the global position
        logger.debug("global_x(m) " +
                    str(round(self.globalPosition[0], 3)) + " global_y(m) " +
                    str(round(self.globalPosition[1], 3) ) )

    def drawVector(self):                                                   # argument is an np array
        vector = self.point - self.globalPosition                           # the vector describing the next step
        self.vectorLength = math.sqrt(vector[0]**2 + vector[1]**2)          # length in m
        self.vectorDirection = math.atan2(vector[1], vector[0])             # discover vector direction
        logger.debug("vectorLength(m) " +
                         str(round(self.vectorLength, 3)) +
                         " vectorDirection(deg) " +
                         str(round(math.degrees(self.vectorDirection), 1)))

    def trajectory(self):
        span = math.radians(5)
        gap = self.vectorDirection - self.heading
        if gap > math.radians(180):                                         # large turns should be reversed
                gap = gap - math.radians(360)
        if gap > span:
            self.flip = 1                                                   # positive turn needed
        elif gap < -span:
            self.flip = -1                                                  # negative turn needed
        else:
            self.flip = 0                                                   # go straight
        logger.debug("CurveFlip " + str(self.flip) )
        return self.flip

    def stackHeading(self):                                                 # increment heading & ensure heading doesn't exceed 180
        self.heading = self.heading + self.angularDisplacement              # update heading by the turn amount executed
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

    def move(self, point):

        self.getWheelIncrements()                                           # get the very first nonzero readings fron enconders
        self.point = np.array(point)
        self.displacement()                                                 # increment the displacements (update robot attributes)
        self.stackDisplacement()                                            # add the new displacement to global position
        self.stackHeading()                                                 # add up the new heading
        self.drawVector()                                                   # draw vector to the destination
        self.trajectory()                                                   # recompute if turning is needed

        logger.debug("START_CURVING " + str(time.monotonic()))

        while abs(self.flip):                                               # flip is +/-1 for turning.  flip is zero when heading points to target
            self.setMotion([self.cruiseRate, self.curveRate * self.flip])   # closed loop command for turning
            self.displacement()                                             # increment the displacements (update robot attributes)
            self.stackDisplacement()                                        # add the new displacement to global position
            self.stackHeading()                                             # add up the new heading
            self.drawVector()                                               # draw vector to the destination
            self.trajectory()                                               # recompute if turning is needed
            self.checkLoop()                                                # calculate how much to sleep
            if self.sleepTime > 0.001:
                time.sleep(self.sleepTime)

        logger.debug("START_DRIVING " + str(time.monotonic()))

        while self.vectorLength > ( self.tolerance ):                       # criteria to stop driving
            self.setMotion([self.cruiseRate, 0])                            # closed loop driving forward
            self.displacement()                                             # update the displacements
            self.stackDisplacement()
            self.stackHeading()
            self.drawVector()
            self.trajectory()
            self.checkLoop()                                                # calculate how much to sleep
            if self.sleepTime > 0.001:
                time.sleep(self.sleepTime)

        logger.debug("SETTLE " + str(time.monotonic()))

        while self.speed != 0 and self.angularVelocity != 0:
            self.setMotion([0, 0])
            print( self.speed,  self.angularVelocity)
            time.sleep(0.035)

        logger.debug("Log_completed " + str(time.monotonic()))
        print("Movements finished. Log file: robotTest.log")
