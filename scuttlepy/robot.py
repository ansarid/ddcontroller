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
        self.globalPosition = [0, 0]

        self.l_motorChannel = 1
        self.r_motorChannel = 2

        self.l_encoderAddress = 0x43
        self.r_encoderAddress = 0x40

        self.wheelBase = 0.201      # L - meters
        self.wheelRadius = 0.041    # R - meters

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

        A = np.array([[     R/2,     R/2],
                      [-R/(2*L), R/(2*L)]])                 # This matrix relates phi dot left and phi dot right to x dot and theta dot.

        B = np.array([self.l_wheel.getSpeed(),
                      self.r_wheel.getSpeed()])

        C = np.matmul(A, B)                                 # Perform matrix multiplication
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

start_time = time.time()

while (time.time() - start_time) <= 5:                                   # Run loop for 10 seconds.

    scuttle.setMotion([0.5, -1])                                           # Set target robot speed and angular velocity [m/s,rad/s]
    print(scuttle.l_wheel.getSpeed(), ",", scuttle.r_wheel.getSpeed())    # Print Wheel Speeds.
