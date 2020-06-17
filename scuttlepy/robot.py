import time
import math
import numpy as np
from scuttlepy import wheels
import logging

# Create and configure logger
logging.basicConfig(filename="robot.log", format='%(asctime)s %(message)s', filemode='w')
logger = logging.getLogger()          # create an object
logger.setLevel(logging.DEBUG)      # set threshold of logger to DEBUG


class SCUTTLE:

    def __init__(self):

        self.speed = 0
        self.heading = 0
        self.compass = 0
        self.angularVelocity = 0
        self.globalPosition = np.array([0, 0])
        self.angularDisp = 0                        # for tracking displacement between waypoints
        self.forwardDisp = 0                        # for tracking displacement between waypoints

        self.l_motorChannel = 1
        self.r_motorChannel = 2

        self.l_encoderAddress = 0x43
        self.r_encoderAddress = 0x40

        self.wheelBase = 0.201                          # L - meters
        self.wheelRadius = 0.041                        # R - meters
        self.wheelIncrements = np.array([0, 0])         # latest increments of wheels

        self.L = self.wheelBase
        self.R = self.wheelRadius

        # For Open Loop Control

        self.rampDown = 0.020                # m
        self.overSteer = math.radians(5)     # deg

        self.batteryVoltage = 0

        self.r_wheel = wheels.Wheel(self.r_motorChannel, self.r_encoderAddress)
        self.l_wheel = wheels.Wheel(self.l_motorChannel, self.l_encoderAddress, invert_encoder=True)

    def setGlobal(self, pos):
        self.globalPosition = pos

    def setHeading(self, heading):
        self.heading = heading

    def getWheelIncrements(self):                                       # get the wheel increment in radians

        self.l_wheel.positionInitial = self.l_wheel.positionFinal                           # transfer previous reading.
        self.r_wheel.positionInitial = self.r_wheel.positionFinal                           # transfer previous reading.

        self.l_wheel.positionFinal = self.l_wheel.encoder.readPos()                         # reading, raw.
        self.r_wheel.positionFinal = self.r_wheel.encoder.readPos()                         # reading, raw.

        wheelIncrements = np.array([self.l_wheel.getTravel(self.l_wheel.positionInitial, self.l_wheel.positionFinal),
                                    self.r_wheel.getTravel(self.r_wheel.positionInitial, self.r_wheel.positionFinal)])        # store wheels travel in radians

        logger.debug("Latest Wheel Increments: "+str(wheelIncrements))

        return wheelIncrements

    def getChassis(self, displacement):                                 # this function returns the chassis displacement
        A = np.array([[          self.R/2,         self.R/2],
                      [-self.R/(2*self.L), self.R/(2*self.L)]])         # This matrix relates [PDL, PDR] to [XD,TD]
        B = displacement                                                # this array should store phi displacements (in radians)
        C = np.matmul(A, B)                                             # perform matrix multiplication
        C = np.round(C, decimals=3)                                     # round the matrix

        return(C)                                                       # returns a matrix containing [dx (m), dTheta (rad)]

    def getChassisVelocity(self):                                       # Forward Kinematics
                                                                        # Function to update and return [x_dot,theta_dot]
        B = np.array([self.l_wheel.speed,                               # make an array of wheel speeds (rad/s)
                      self.r_wheel.speed])

        C = self.getChassis(B)                                          # Perform matrix multiplication
        self.speed = C[0]                                               # Update speed of SCUTTLE [m/s]
        self.angularVelocity = C[1]                                     # Update angularVelocity = [rad/s]

        return [self.speed, self.angularVelocity]                       # return [speed, angularVelocity]

    def setMotion(self, targetMotion):                                  # Take chassis speed and command wheel speeds
                                                                        # argument: [x_dot, theta_dot]
        print("Target: ", targetMotion)
        L = self.wheelBase
        R = self.wheelRadius

        A = np.array([[ 1/R, -L/R],                                     # This matrix relates chassis to wheels
                      [ 1/R,  L/R]])

        B = np.array([targetMotion[0],                                  # Create an array for chassis speed
                      targetMotion[1]])

        C = np.matmul(A, B)                                             # Perform matrix multiplication

        logger.debug("Closed Loop Phi Targets: "+str(C))                # indicate wheelspeed targets in log

        self.l_wheel.setAngularVelocity(C[0])                           # Set angularVelocity = [rad/s]
        self.r_wheel.setAngularVelocity(C[1])                           # Set angularVelocity = [rad/s]

    def displacement(self):
        chassisIncrement = self.getChassis(self.getWheelIncrements())     # get latest chassis travel (m, rad)
        self.forwardDisp += chassisIncrement[0]                           # add the latest advancement(m) to the total
        self.angularDisp += chassisIncrement[1]

    def resetDisp(self):
        self.angularDisp = 0                                            # reset the attribute for counting up angular displacement
        self.forwardDisp = 0                                            # reset the attribute for counting up forward displacement

    def move(self, point):

        def calculateTurn(vectorDirection):
            turn = vectorDirection - self.heading                       # calculate required turn, rad
            if turn > math.radians(180):                                # large turns should be reversed
                turn = turn - math.radians(360)
            return turn

        def getTurnDirection(val):                                      # check direction of turn and initiate turning
            if val == 0:
                self.setMotion([0, 0])
            elif val > 0:
                self.setMotion([0,  1.5])
            elif val < 0:
                self.setMotion([0, -1.5])

        def trimHeading():                                              # ensure heading doesn't exceed 360
            if self.heading > math.pi:
                self.heading += (2 * math.pi)
            if self.heading < -math.pi:
                self.heading += (2 * math.pi)

        # def generateCurve(myTurn):
        #     alpha = vectorDirection - self.heading      # alpha is the curve amount
        #     L2 = abs(curveRadius * math.tan(alpha / 2)) # abs for right hand turns
        #     arcLen = curveRadius * alpha                # the arc length of the curve, meters
        #     return alpha

        # populate encoder values for startup
        self.getWheelIncrements()   # get the very first nonzero readings fron enconders

        # initialize variables at zero
        # x = 0                                   # x
        # rotation = 0                            # rotation along theta direction

        vector = point - self.globalPosition    # the vector describing the next step

        vectorLength = math.sqrt(vector[0]**2 + vector[1]**2)   # length in m

        vectorDirection = math.atan2(vector[1], vector[0])

        myTurn = calculateTurn(vectorDirection)

        stopped = False

        getTurnDirection(myTurn)                 # myTurn argument is for choosing direction and initiating the turn

        # ---------------FIRST STEP, TURN HEADING---------------------------------------------------------------------
        self.resetDisp()                                       # reset displacements

        rotation_low = int(100*(myTurn - self.overSteer))      # For defining acceptable range for turn accuracy.
        rotation_high = int(100*(myTurn + self.overSteer))     # Needs to be redone with better solution

        print("Turning.")

        while True:                         # Needs to be turned into a dp while loop instead of while break.
            self.displacement()             # increment the displacements (update robot attributes)

            print("turning, (deg):", round(math.degrees(self.angularDisp), 2), " \t Target:", math.degrees(myTurn))       # print theta in radians
            time.sleep(0.08)

            if int(self.angularDisp*100) in range(rotation_low, rotation_high):      # check if we reached our target range
                self.setMotion([0, 0])
                if not stopped:
                    stopTime = time.time()
                    stopped = True
                if (time.time() - stopTime) > 0.200:        # give 200 ms for turning to settle
                    break

        print("Turning completed.")
        self.heading = self.heading + self.angularDisp      # update heading by the turn amount executed

        print("angularDisp:", int(math.degrees(self.angularDisp)), "   Heading:", round(math.degrees(self.heading),1))
        print("__DRIVING__")

        # ---------------SECOND STEP, DRIVE FORWARD-------------------------------------------------------------
        self.resetDisp()            # reset displacements
        stopped = False             # reset the stopped flag

        # NEED TO MODIFY THIS COMMAND TO CONTINUOUSLY DRIVE IN CLOSED LOOP MANNER
        self.setMotion([0.15, 0])    # begin driving forward 

        while True:
            self.displacement()     # update the displacements

            print("ForwardDisp(m):", round(self.forwardDisp, 3), "\t\tTarget distance:", vectorLength)                   # print x in meters
            time.sleep(0.08)

            if self.forwardDisp > (vectorLength-self.rampDown):
                self.setMotion([0, 0])
                if not stopped:
                    stopTime = time.time()
                    stopped = True
                    # print("Stopping Forward")
                if (time.time() - stopTime) > 0.200:
                    break

        myMovementX = self.forwardDisp * math.cos(self.heading)
        myMovementY = self.forwardDisp * math.sin(self.heading)
        self.globalPosition = self.globalPosition + np.array([myMovementX, myMovementY])           # update the position of robot in global frame

        print("driving completed (m):", round(self.forwardDisp, 3))
        self.resetDisp()        # reset displacements


        print("x advanced:", round(myMovementX, 3), "  y advanced:", round(myMovementY, 3), "  global pos:", np.round(self.globalPosition, 3))

