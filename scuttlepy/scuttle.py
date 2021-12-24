#!/usr/bin/python3

import os
import yaml
import time
import threading
import numpy as np
import RPi.GPIO as GPIO

from .constants import *
from . import wheels

def parse_yaml_config(config):
    # Validate the config file name
    if config is None:
        raise Exception("Config file None.")

    try:
        # Parse the YAML config. Expand any environment variables
        # inside, prior ro parsing.
        with open(config, 'r') as f:
            y = yaml.safe_load(os.path.expandvars(f.read()))
    except IOError:
        raise Exception("Could not read config file {}.".format(config))

    #print(yaml.dump(y))

    # Validate the YAML specification
    try:
        if 'scuttle' not in y:
            raise Exception("'scuttle' section not present")
        elif 'chassis' not in y['scuttle']:
            raise Exception("'chassis' section not present")

        chassis = y['scuttle']['chassis']

        if 'wheels' not in chassis:
            raise Exception("'chassis' section not present")

        wheels = chassis['wheels']

        if 'rl_wheel' not in wheels:
            raise Exception("'rl_wheel' section not present")
        elif 'rr_wheel' not in wheels:
            raise Exception("'rr_wheel' section not present")

        rl_wheel = wheels['rl_wheel']
        rr_wheel = wheels['rr_wheel']

        if 'motor_control_gpio' not in rl_wheel:
            raise Exception("'motor_control_gpio' section not present in'rl_wheel'")
        elif 'motor_control_gpio' not in rr_wheel:
            raise Exception("'rr_wheel' section not present in'rr_wheel'")
    except:
        raise

    return y

class SCUTTLE:

    def __init__(self, config=None, openLoop=False):

        GPIO.setmode(GPIO.BOARD)

        try:
            y = parse_yaml_config(config)
        except IOError:
            print("Config file parse failed.")
            raise

        #print(yaml.dump(y))

        self.heading = 0
        self.velocity = 0
        self.angularVelocity = 0
        self.globalPosition = [0, 0]

        chassisdB = y['scuttle']['chassis']
        wheelsdB  = chassisdB['wheels']
        rl_wheel  = wheelsdB['rl_wheel']
        rl_gpio   = rl_wheel['motor_control_gpio']
        rl_invert = rl_wheel['invert']
        rr_wheel  = wheelsdB['rr_wheel']
        rr_gpio   = rr_wheel['motor_control_gpio']
        rr_invert = rr_wheel['invert']

        # Get the wheel base from the config specification. If not
        # present then use the default value
        self.wheelBase = chassisdB.get('wheel_base', WHEEL_BASE)

        # Get the wheel radius from the config specification. If not
        # present then use the default value
        self.wheelRadius = chassisdB.get('wheel_radius', WHEEL_RADIUS)

        # Get the i2c bus id from the config specification. If not
        # present then use the default value
        bus = wheelsdB.get('i2c_bus_id', I2C_BUS)

        self.leftEncoderAddress = rl_wheel.get('encoder_addr', LEFT_ENCODER_ADDRESS)
        self.leftMotorChannel   = (rl_gpio['pwm'], rl_gpio['digital'])
        self.leftWheel          = wheels.Wheel(bus,
                                               self.leftMotorChannel,    # Create left wheel object
                                               self.leftEncoderAddress,
                                               invertEncoder=rl_invert,
                                               invertMotor=rl_invert,
                                               openLoop=openLoop,
                                               )

        self.rightEncoderAddress = rr_wheel.get('encoder_addr', RIGHT_ENCODER_ADDRESS)
        self.rightMotorChannel   = (rr_gpio['pwm'], rr_gpio['digital'])
        self.rightWheel          = wheels.Wheel(bus,
                                                self.rightMotorChannel,  # Create right wheel object
                                                self.rightEncoderAddress,
                                               invertEncoder=rr_invert,
                                               invertMotor=rr_invert,
                                                openLoop=openLoop,
                                                )

        self.wheelSpeeds          = [0, 0]                      # [Left wheel speed, Right wheel speed.]
        self.targetMotion         = [0, 0]
        self._loopStart           = time.monotonic_ns()         # Updates when we grab chassis displacements
        self._timeInitial         = time.monotonic_ns()
        self._timeFinal           = 0
        self._angularDisplacement = 0                           # For tracking displacement between waypoints
        self._forwardDisplacement = 0                           # For tracking displacement between waypoints
        self._wheelIncrements     = np.array([0, 0])            # Latest increments of wheels

        self.stopped = False

        self._loopFreq = 50                                             # Target Wheel Loop frequency (Hz)
        self._wait = 1/self._loopFreq                                   # Corrected wait time between encoder measurements (s)

        self.wheelsThread = threading.Thread(target=self._wheelsLoop)   # Create wheel loop thread object
        self.wheelsThread.start()                                       # Start wheel loop thread object

    def _wheelsLoop(self):

        while not self.stopped:

            startTime = time.monotonic_ns()                                 # Record loop start time

            self.leftWheel.update()                                         # Update left wheel readings
            self.rightWheel.update()                                        # Update right wheel readings

            self.velocity, self.angularVelocity = self.getMotion()          # Get scuttle linear and angular velocities

            leftWheelTravel = self.leftWheel.getTravel()                    # Get left wheel travel
            rightWheelTravel = self.rightWheel.getTravel()                  # Get right wheel travel

            wheelbaseTravel = (leftWheelTravel + rightWheelTravel)/2        # Calculate wheel displacement

            self.globalPosition = [self.globalPosition[0]+(wheelbaseTravel*np.cos(self.heading)),   # Calculate global X position
                                   self.globalPosition[1]+(wheelbaseTravel*np.sin(self.heading))    # Calculate global Y position
                                   ]

            self.heading += ((rightWheelTravel - leftWheelTravel)/(self.wheelBase))       # Calculate global heading

            time.sleep(sorted([self._wait-((time.monotonic_ns()-startTime)/1e9), 0])[1])    # Measure time since start and subtract from sleep time
            # print((time.monotonic_ns()-startTime)/1e6)                                      # Print loop time in ms

        self.rightWheel.stop()                                  # Once wheels thread loop has broken, stop right wheel
        self.leftWheel.stop()                                   # Once wheels thread loop has broken, stop left wheel

    def stop(self):                                             # Stop SCUTTLE
        self.setMotion([0, 0])                                  # Set linear and angular velocity to 0
        self.stopped = True                                     # Set stopped flag to True
        self.wheelsThread.join()                                # Wait for the wheels thread to stop

    def setGlobalPosition(self, pos):                           # Set global position
        self.globalPosition = pos                               # Set global position to desired position
        return self.globalPosition                              # return new global position

    def setHeading(self, heading):                              # Set global heading
        self.heading = heading                                  # Set heading to desired heading
        return self.heading                                     # return new global heading

    def getGlobalPosition(self):                                # get global position
        return self.globalPosition                              # return global position

    def getHeading(self):                                       # get global heading
        return self.heading                                     # return global heading

    def getLinearVelocity(self):                                # get linear velocity
        return self.velocity                                    # return linear velocity

    def getAngularVelocity(self):                               # get angular velocity
        return self.angularVelocity                             # return angular velocity

    def setMotion(self, targetMotion):                          # Take chassis speed and command wheels
                                                                # argument: [x_dot, theta_dot]
        # self.targetMotion = targetMotion

        L = self.wheelBase/2
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
        L = self.wheelBase/2
        R = self.wheelRadius

        A = np.array([[     R/2,     R/2],                      # This matrix relates [PDL, PDR] to [XD,TD]
                      [-R/(2*L), R/(2*L)]])

        B = np.array([self.leftWheel.getAngularVelocity(),      # make an array of wheel speeds (rad/s)
                      self.rightWheel.getAngularVelocity()])

        C = np.matmul(A, B)                                     # perform matrix multiplication

        self.velocity = C[0]                                    # Update speed of SCUTTLE [m/s]
        self.angularVelocity = C[1]                             # Update angularVelocity = [rad/s]

        return [self.velocity, self.angularVelocity]            # return [speed, angularVelocity]
