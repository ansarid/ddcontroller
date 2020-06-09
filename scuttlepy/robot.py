import time
import math
import numpy as np
from scuttlepy import wheels


class SCUTTLE:

    def __init__(self):

        self.speed = 0
        self.heading = 0
        self.compass = 0
        self.angularVelocity = 0
        self.globalPosition = np.array([0, 0])

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

        ######################################################

        self.batteryVoltage = 0

        self.r_wheel = wheels.Wheel(self.r_motorChannel, self.r_encoderAddress)
        self.l_wheel = wheels.Wheel(self.l_motorChannel, self.l_encoderAddress, invert_encoder=True)

    def setGlobal(self, pos):
        self.globalPosition = pos

    def setHeading(self, heading):
        self.heading = heading

    def getChassis(self, disp):                                         # this function returns the chassis displacement
        A = np.array([[          self.R/2,         self.R/2],
                      [-self.R/(2*self.L), self.R/(2*self.L)]])         # This matrix relates [PDL, PDR] to [XD,TD]
        B = disp                                                        # this array should store phi displacements (in radians)
        C = np.matmul(A, B)                                             # perform matrix multiplication
        C = np.round(C, decimals=3)                                     # round the matrix
        return(C)                                                       # returns a matrix containing [dx, dTheta]


    def getWheelIncrements(self):

        self.l_wheel.positionInitial = self.l_wheel.positionFinal                           # transfer previous reading.
        self.r_wheel.positionInitial = self.r_wheel.positionFinal                           # transfer previous reading.

        self.l_wheel.positionFinal = self.l_wheel.encoder.readPos()                         # reading, raw.
        self.r_wheel.positionFinal = self.r_wheel.encoder.readPos()                         # reading, raw.

        wheelIncrements = np.array([self.l_wheel.getTravel(self.l_wheel.positionInitial, self.l_wheel.positionFinal),
                                    self.r_wheel.getTravel(self.r_wheel.positionInitial, self.r_wheel.positionFinal)])        # store wheels travel in degrees

        return wheelIncrements

    def getChassisVelocity(self):                                   # Forward Kinematics
                                                           # Function to update and return [x_dot,theta_dot]
        L = self.wheelBase
        R = self.wheelRadius

        A = np.array([[     R/2,    R/2],
                      [-R/(2*L), R/(2*L)]])                 # This matrix relates phi dot left and phi dot right to x dot and theta dot.

        B = np.array([self.l_wheel.speed,
                      self.r_wheel.speed])

        C = np.matmul(A, B)   # Perform matrix multiplication
        self.speed = C[0]                                   # Update speed of SCUTTLE [m/s]
        self.angularVelocity = C[1]                         # Update angularVelocity = [rad/s]

        return [self.speed, self.angularVelocity]           # return [speed, angularVelocity]

    def setMotion(self, targetMotion):                      # Inverse Kinematics
                                                            # Function to update and return [phi_dot_left, phi_dot_right]
        L = self.wheelBase
        R = self.wheelRadius


        A = np.array([[ 1/R, -L/R],
                      [ 1/R,  L/R]])                        # This matrix relates

        B = np.array([targetMotion[0],
                      targetMotion[1]])

        C = np.matmul(A, B)                                 # Perform matrix multiplication

        self.l_wheel.setSpeed(C[0])                         # Set speed of SCUTTLE [m/s]
        self.r_wheel.setSpeed(C[1])                         # Set angularVelocity = [rad/s]



    def move(self, point):

        def goStraight():
            self.l_wheel.motor.setDuty(0.735)
            self.r_wheel.motor.setDuty(0.75)

        def turnL():
            self.l_wheel.motor.setDuty(-0.63)
            self.r_wheel.motor.setDuty(0.65)

        def turnR():
            self.l_wheel.motor.setDuty(0.63)
            self.r_wheel.motor.setDuty(-0.65)

        def straightLine(myDistance):
            goStraight()

        def stop():
            self.l_wheel.motor.setDuty(0)
            self.r_wheel.motor.setDuty(0)

        def calculateTurn(theta12):

            turn = theta12 - self.heading                # calculate required turn, rad
            turn = math.degrees(turn)                       # convert to degrees

            if turn > 180:                                  # large turns should be reversed
                turn = turn - 180 * -1
            myTurn = math.radians(turn)

            return myTurn

        def turn(val):
            if val > 0:
                turnL()
            else:
                turnR()

        # initialize variables at zero
        x = 0                                   # x
        t = 0                                   # theta

        myVector = point - self.globalPosition

        vectorLength = math.sqrt(myVector[0]**2 + myVector[1]**2) # length in m

        theta12 = math.atan2(myVector[1], myVector[0])

        myTurn = calculateTurn(theta12)

        myDistance = vectorLength       # m

        stopped = False

        turn(myTurn)                            # myTurn argument is for choosing direction and initiating the turn

        # ---------------FIRST STEP, TURN HEADING---------------------------------------------------------------------
        # this loop continuously adds up the x forward movement originating from the encoders.

        t_low = int(1000*(myTurn - self.overSteer))
        t_high = int(1000*(myTurn + self.overSteer))

        print("Turning.")

        while True:
            chassisIncrement = self.getChassis(self.getWheelIncrements())            # get latest chassis travel
            x = x + chassisIncrement[0]                 # add the latest advancement(m) to the total
            t = t + chassisIncrement[1]

            print("turning, deg):", round(math.degrees(t), 2), "\tTarget:", math.degrees(myTurn))       # print theta in radians
            time.sleep(0.08)

            if int(t*1000) in range(t_low, t_high):      # check if we reached our target range
                stop()
                if not stopped:
                    stopTime = time.time()
                    stopped = True
                    print("Stopping Turning.")
                if (time.time() - stopTime) > 0.200:
                    break
        print("turning completed.")
        print("heading:", self.heading)

        # ---------------SECOND STEP, DRIVE FORWARD-------------------------------------------------------------
        # this loop continuously adds up the x forward movement originating from the encoders.

        stopped = False     # reset the stopped flag
        goStraight()        # begin the driving forward

        print("Driving Forward.")

        while True:
            chassisIncrement = self.getChassis(self.getWheelIncrements())            # get latest chassis travel
            x = x + chassisIncrement[0]                 # add the latest advancement(m) to the total
            t = t + chassisIncrement[1]

            print("x(m)", round(x,3), "\t\tTarget:", myDistance )                        # print x in meters
            time.sleep(0.08)

            if x > (myDistance-self.rampDown):
                stop()
                if not stopped:
                    stopTime = time.time()
                    stopped = True
                    print("Stopping Forward")
                if (time.time() - stopTime) > 0.200:
                    break
        self.globalPosition = np.array(point)
        self.heading = myTurn
        # self.heading = 0

waypoints = [
            [   0,  0.2],
            [ 0.2,  0.2],
            [ 0.2, -0.2],
            [-0.2, -0.2],
            [-0.2,  0.2],
            [   0,  0.2],
            [   0,    0],
            ]

scuttle = SCUTTLE()

for waypoint in waypoints:
    print("DRIVING TO POINT", waypoint)
    scuttle.move(waypoint)
    print("COMPLETED POINT", waypoint)
