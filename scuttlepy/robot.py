import time
import math
import logging
import numpy as np
from scuttlepy import gpio
from scuttlepy import wheels
from scuttlepy import mpu



# Create and configure logger
logging.basicConfig(filename="robotTest.log", format='%(asctime)s %(message)s', filemode='w')
logger = logging.getLogger()                                                # create an object
logger.setLevel(logging.DEBUG)                                              # set threshold of logger to DEBUG
logger.debug("ColumnA ColumnB ColumnC ColumnD")

gpio.write(1, 3, 0)


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

        self.l_encoderAddress = 0x43
        self.r_encoderAddress = 0x40

        # self.wheelBase = 0.201                                              # L - meters
        self.wheelBase = 0.180                                              # L - meters
        # self.wheelBase = 0.170                                              # L - meters

        self.wheelRadius = 0.041                                            # R - meters
        self.wheelIncrements = np.array([0, 0])                             # latest increments of wheels

        self.L = self.wheelBase
        self.R = self.wheelRadius

        self.rampDown = 0.020                                               # m
        self.overSteer = math.radians(10)                                   # deg

        self.cruiseRate = 0.150                                             # fwd driving speed, m/s
        self.curveRadius = 0.300                                            # curve radius (m)
        self.curveRate = self.cruiseRate / self.curveRadius                 # curve rotational speed (rad/s)
        self.L2 = 0                                                         # amount to cut from straight path
        self.arcLen = 0

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

        self.l_wheel.positionFinal = self.l_wheel.encoder.readPos()         # reading, raw.
        self.r_wheel.positionFinal = self.r_wheel.encoder.readPos()         # reading, raw.

        wheelIncrements = np.array([self.l_wheel.getTravel(self.l_wheel.positionInitial,
                                                           self.l_wheel.positionFinal),
                                    self.r_wheel.getTravel(self.r_wheel.positionInitial,
                                                           self.r_wheel.positionFinal)])        # store wheels travel in radians

        logger.debug("Latest_Wheel_Increments: " + str(wheelIncrements[0]) + " " + str(wheelIncrements[1]))

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

        logger.debug("PhiTargetLeft " + str(round(C[0],3)))                 # indicate wheelspeed targets in log
        logger.debug("PhiTargetRight " + str(round(C[1],3)))                # indicate wheelspeed targets in log

        self.l_wheel.setAngularVelocity(C[0])                               # Set angularVelocity = [rad/s]
        self.r_wheel.setAngularVelocity(C[1])                               # Set angularVelocity = [rad/s]

    def displacement(self):

        chassisIncrement = self.getChassis(self.getWheelIncrements())       # get latest chassis travel (m, rad)
        self.forwardDisplacement += chassisIncrement[0]                     # add the latest advancement(m) to the total
        self.angularDisplacement += chassisIncrement[1]                     # add the latest advancement(rad) to the total

        logger.debug("Chassis_Increment(m,rad) " +
                    str(round(chassisIncrement[0], 3)) + " " +
                    str(round(chassisIncrement[1], 3)) + " " +
                    str(time.time()))

        logger.debug("Gyro_raw(deg/s) " +
            str(round(self.imu.readAll()['gyro'][2], 3)) + " " +
            str(time.time()))

    def resetDisplacement(self):

        self.angularDisplacement = 0                                        # reset the attribute for counting up angular displacement
        self.forwardDisplacement = 0                                        # reset the attribute for counting up forward displacement

    def move(self, point, point2=none):

        def calculateTurn(vectorDirection):
            turn = vectorDirection - self.heading                           # calculate required turn, rad
            if turn > math.radians(180):                                    # large turns should be reversed
                turn = turn - math.radians(360)
            return turn

        def getTurnDirection(self, val):                                    # check direction of turn and initiate turning
            if val == 0:
                self.turnRate = 0
            elif val > 0:
                self.turnRate = 0.3                                         # rad/s
            elif val < 0:
                self.turnRate = -0.3                                        # rad/s

        def trimHeading(self):                                              # ensure heading doesn't exceed 360
            if self.heading > math.pi:
                self.heading += (2 * math.pi)
            if self.heading < -math.pi:
                self.heading += (2 * math.pi)

        def generateCurve(vectorDirection2):
            #alpha = vectorDirection2 - self.heading                       # alpha is the curve amount
            self.L2 = abs(self.curveRadius * math.tan(alpha / 2))         # abs for right hand turns
            self.arcLen = self.curveRadius * alpha                             # the arc length of the curve, meters
            return arcLen

        self.getWheelIncrements()                                           # get the very first nonzero readings fron enconders

        vector = point - self.globalPosition                                # the vector describing the next step

        vectorLength = math.sqrt(vector[0]**2 + vector[1]**2)               # length in m

        vectorDirection = math.atan2(vector[1], vector[0])                  # discover vector direction

        myTurn = calculateTurn(vectorDirection)                             # discover required turning (rad)

        getTurnDirection(self, myTurn)                                      # myTurn argument is for choosing direction and initiating the turn

        #________SECOND VECTOR__________
        
        vector2 = point2 - point
        
        vectorDirection2 = math.atan2(vector2[1], vector2[0])
        
        myTurn2 = vectorDirection2 - vectorDirection    # turn amount, radians
        
        self.arcLen = generateCurve(myTurn2) # arc length will be criteria for finishing curve
        
        
        # ---------------FIRST STEP, TURN HEADING---------------------------------------------------------------------

        self.resetDisplacement()                                            # reset displacements
        stopped = False                                                     # reset the stopped flag

        logger.debug("Stopped_Flag_Low START_TURNING " + str(time.time()))

        rotation_low = int(100*(myTurn - self.overSteer))                   # For defining acceptable range for turn accuracy.
        rotation_high = int(100*(myTurn + self.overSteer))                  # Needs to be redone with better solution

        print("START_TURNING")

        while True:                                                         # Needs to be turned into a do while loop instead of while break.

            self.setMotion([0, self.turnRate])                              # closed loop command for turning
            self.displacement()                                             # increment the displacements (update robot attributes)
            logger.debug("Turning_Displacement(deg) " +
                         str(round(math.degrees(self.angularDisplacement), 1)) +
                         " Target(deg) " +
                         str(round(math.degrees(myTurn), 1)))
            time.sleep(0.035)                                               # aim for 100ms loops

            if int(self.angularDisplacement*100) in range(rotation_low, rotation_high):     # check if we reached our target range
                self.setMotion([0, 0])
                gpio.write(1, 3, 1)

                logger.debug("Settling_Displacement(deg) " +
                             str(round(math.degrees(self.angularDisplacement), 1)) +
                             " Target(deg) " +
                             str(round(math.degrees(myTurn), 1)))

                self.turnRate = 0                                           # maintain turnRate 0 for possible overshoot
                if not stopped:
                    stopTime = time.time()
                    stopped = True
                    logger.debug("Stopped_Flag_High" + " FINISH_TURNING " + str(time.time()))
                if (time.time() - stopTime) > 0.200:                        # give 200 ms for turning to settle
                    break

        logger.debug("TURN_COMPLETED " + str(time.time()))

        self.heading = self.heading + self.angularDisplacement              # update heading by the turn amount executed

        logger.debug("Angular_displacement " +
                     str(round(math.degrees(self.angularDisplacement), 1)))         # degrees rounded to 1 decimal

        logger.debug("Heading " +
                     str(round(math.degrees(self.heading), 1)))             # degrees rounded to 1 decimal

        # ---------------SECOND STEP, DRIVE FORWARD-------------------------------------------------------------

        self.resetDisplacement()                                            # reset displacements
        self.cruiseRate = 0.15                                              # m/s
        stopped = False                                                     # reset the stopped flag

        logger.debug("Stopped_Flag_Low START_DRIVING " + str(time.time()))

        print("START DRIVING")
        self.setMotion([0, self.turnRate])                                  # closed loop command for turning

        while True:

            self.setMotion([self.cruiseRate, 0])                            # closed loop driving forward
            self.displacement()                                             # update the displacements

            logger.debug("Forward_Displacement(m) " +
                         str(round(self.forwardDisplacement, 3)) +
                         " Target_Distance(m) " +
                         str(vectorLength))

            time.sleep(0.035)                                               # aiming for 100ms loop0?

            if self.forwardDisplacement > (vectorLength - self.rampDown):
                self.cruiseRate = 0                                         # ensure target speed stays at zero
                self.setMotion([self.cruiseRate, 0])
                if not stopped:
                    stopTime = time.time()
                    stopped = True
                    logger.debug("Stopped_Flag_High STOP_DRIVING " + str(time.time()))
                if (time.time() - stopTime) > 0.200:
                    break

        myMovementX = self.forwardDisplacement * math.cos(self.heading)
        myMovementY = self.forwardDisplacement * math.sin(self.heading)
        self.globalPosition = self.globalPosition + np.array([myMovementX, myMovementY])           # update the position of robot in global frame

        logger.debug("Distance_Achieved(m) " + str(round(self.forwardDisplacement, 3)))
        self.resetDisplacement()        # reset displacements

        logger.debug("Advancement_x " + str(round(myMovementX,3)) + " Advancement_y " + str(round(myMovementY,3)))
        logger.debug("Global_Position " + str(round(self.globalPosition[0],3)) + " " + str(round(self.globalPosition[1],3)))
        logger.debug("Log_completed " + str(time.time()))
        print("Move finished. Log file: robotTest.log")

# ---------------THIRD STEP, CURVE THEN STOP-------------------------------------------------------------

        self.resetDisplacement()                                            # reset displacements
        self.cruiseRate = 0.15                                              # m/s
        stopped = False                                                     # reset the stopped flag

        logger.debug("Stopped_Flag_Low START_CURVING " + str(time.time()))

        print("START CURVING")
        self.setMotion([self.cruiseRate, self.curveRate])                   # closed loop command for curving

        while True:

            self.setMotion([self.cruiseRate, self.curveRate])               # closed loop driving forward
            self.displacement()                                             # update the displacements

            logger.debug("Curve_Fwd_Displacement(m) " +
                         str(round(self.forwardDisplacement, 3)) +
                         " Target_Distance(m) " +
                         str(self.arcLen))

            time.sleep(0.035)                                               # aiming for 100ms loop0?

            if self.forwardDisplacement > (self.arcLen - self.rampDown):
                self.cruiseRate = 0                                         # ensure target speed stays at zero
                self.setMotion([self.cruiseRate, self.curveRate])
                if not stopped:
                    stopTime = time.time()
                    stopped = True
                    logger.debug("Stopped_Flag_High STOP_CURVING " + str(time.time()))
                if (time.time() - stopTime) > 0.200:
                    break

        #self.globalPosition = self.globalPosition + np.array([myMovementX, myMovementY])           # update the position of robot in global frame

        logger.debug("Curve_Distance_Achieved(m) " + str(round(self.forwardDisplacement, 3)))
        self.resetDisplacement()        # reset displacements
        
        logger.debug("Log_completed " + str(time.time()))
        print("Move finished. Log file: robotTest.log")
