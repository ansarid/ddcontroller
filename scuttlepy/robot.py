import time
import math
import numpy as np
from scuttlepy.actuation import wheels


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

        self.wheelBase = 0.201      # L - meters
        self.wheelRadius = 0.041    # R - meters

        self.L = self.wheelBase
        self.R = self.wheelRadius

        self.batteryVoltage = 0

        self.r_wheel = wheels.Wheel(self.r_motorChannel, self.r_encoderAddress)
        self.l_wheel = wheels.Wheel(self.l_motorChannel, self.l_encoderAddress, invert_encoder=True)

    def setGlobal(self, pos):
        self.globalPosition = pos

    def setHeading(self, heading):
        self.heading = heading

    def getMotion(self):                                   # Forward Kinematics
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

    # def move(self, pos)
    # def rotate(self, theta)


scuttle = SCUTTLE()


myPoint = np.array([-0.5, 0.5])                    # x, y destination in meters
myVector = myPoint - scuttle.globalPosition
print("myVector:", myVector)                    # in meters, x & y

vectorLength = math.sqrt(myVector[0]**2 + myVector[1]**2) # length in m

theta12 = math.atan2(myVector[1], myVector[0])
print("theta12, rad:", theta12)
turn = theta12 - scuttle.heading                # calculate required turn, rad
turn = math.degrees(turn)                       # convert to degrees
print("turn, deg:", turn)
if( turn > 180):                                # large turns should be reversed
    turn = turn -180 * -1                       # 

print("myDistance", vectorLength) # how far will scuttle advance (m)

myDistance = vectorLength       # m
rampDown = 0.020                # m
myTurn = math.radians(turn)     # deg
overSteer = math.radians(5)     # deg


def goStraight():
    scuttle.l_wheel.motor.setDuty(0.735)
    scuttle.r_wheel.motor.setDuty(0.75)

def turnL():
    scuttle.l_wheel.motor.setDuty(-0.63)
    scuttle.r_wheel.motor.setDuty(0.65)

def turnR():
    scuttle.l_wheel.motor.setDuty(0.63)
    scuttle.r_wheel.motor.setDuty(-0.65)

def turn(val):
    if val > 0:
        turnL()
    else:
        turnR()

    print("Set DC")


def straightLine(myDistance):
    goStraight()


def stop():
    scuttle.l_wheel.motor.setDuty(0)
    scuttle.r_wheel.motor.setDuty(0)


def getTravel(deg0, deg1):                  # calculate the delta on Left wheel
    trav = deg1 - deg0                      # reset the travel reading
    if((-trav) >= gap):                     # if movement is large (has rollover)
        trav = (deg1 - deg0 + roll)         # forward rollover
    if(trav >= gap):
        trav = (deg1 - deg0 - roll)         # reverse rollover
    return(trav)


def getChassis(disp):                       # this function returns the chassis displacement
    B = disp                                # this array should store phi displacements (in radians)
    C = np.matmul(A, B)                     # perform matrix multiplication
    C = np.round(C, decimals=3)             # round the matrix
    return(C)                               # returns a matrix containing [dx, dTheta]


# define kinematics
R = 0.041                                   # radius in meters
L = 0.201                                   # half of wheelbase meters
res = (360/2**14)                           # resolution of the encoders
roll = int(360/res)                         # variable for rollover logic
gap = 0.5 * roll                            # degress specified as limit for rollover
A = np.array([[R/2, R/2], [-R/(2*L), R/(2*L)]])     # This matrix relates [PDL, PDR] to [XD,TD]

# initialize variables at zero
x = 0                                       # x
t = 0                                       # theta
encL1 = 0
encR1 = 0

stopped = False

turn(myTurn) # myTurn argument is for choosing direction, starts turning

def trackMovement():
    encL0 = encL1                           # transfer previous reading.
    encR0 = encR1                           # transfer previous reading.
    # encoders = [scuttle.l_wheel.getSpeed(), scuttle.r_wheel.getSpeed()]                   # grabs the current encoder readings, raw
    encL1 = round(scuttle.l_wheel.encoder.readPos(), 1)           # reading, raw.
    encR1 = round(scuttle.r_wheel.encoder.readPos(), 1)           # reading, raw.

    # ---- movement calculations
    travL = getTravel(encL0, encL1) * res   # grabs travel of left wheel, degrees
    # travL = -1 * travL                    # this wheel is inverted from the right side
    travR = getTravel(encR0, encR1) * res   # grabs travel of right wheel, degrees

    # build an array of wheel travels in rad/s
    travs = np.array([travL, travR])        # store wheels travel in degrees
    travs = travs * 0.5                     # pulley ratio = 0.5 wheel turns per pulley turn
    travs = travs * 3.14 / 180              # convert degrees to radians
    travs = np.round(travs, decimals=3)     # round the array

    chass = getChassis(travs)               # convert the wheel travels to chassis travel
    x = x + chass[0]                        # add the latest advancement(m) to the total
    t = t + chass[1]

# this loop continuously adds up the x forward movement originating from the encoders.
while True:
    trackMovement()
    # encL0 = encL1                           # transfer previous reading.
    # encR0 = encR1                           # transfer previous reading.
    # # encoders = [scuttle.l_wheel.getSpeed(), scuttle.r_wheel.getSpeed()]                   # grabs the current encoder readings, raw
    # encL1 = round(scuttle.l_wheel.encoder.readPos(), 1)           # reading, raw.
    # encR1 = round(scuttle.r_wheel.encoder.readPos(), 1)           # reading, raw.

    # # ---- movement calculations
    # travL = getTravel(encL0, encL1) * res   # grabs travel of left wheel, degrees
    # # travL = -1 * travL                    # this wheel is inverted from the right side
    # travR = getTravel(encR0, encR1) * res   # grabs travel of right wheel, degrees

    # # build an array of wheel travels in rad/s
    # travs = np.array([travL, travR])        # store wheels travel in degrees
    # travs = travs * 0.5                     # pulley ratio = 0.5 wheel turns per pulley turn
    # travs = travs * 3.14 / 180              # convert degrees to radians
    # travs = np.round(travs, decimals=3)     # round the array

    # chass = getChassis(travs)               # convert the wheel travels to chassis travel
    # x = x + chass[0]                        # add the latest advancement(m) to the total
    # t = t + chass[1]
    # scuttle.heading = t

    print("turn, deg):", math.degrees(t))                              # print theta in radians
    time.sleep(0.08)
    # if x > (myDistance-rampDown) and not stopped:
    #     stop()
    #     stopped = True
    #     print("Stopping")

    t_low = int(100*(myTurn - overSteer))
    t_high = int(100*(myTurn + overSteer))
    # print(t_low, t_high, int(t*100))
    if int(t*100) in range(t_low, t_high):
        stop()
        if not stopped:
            stopTime = time.time()
            stopped = True
        print("Stopping")
        if (time.time() - stopTime) > 0.200:
            break

print("turning completed.")
print("heading:", scuttle.heading)

stopped = False

goStraight() # begin the driving forward

# this loop continuously adds up the x forward movement originating from the encoders.
while True:
    trackMovement()
    print("x(m)", x)                        # print x in meters

    time.sleep(0.08)
    if x > (myDistance-rampDown) and not stopped:
        stop()
        stopped = True
        print("Stopping")