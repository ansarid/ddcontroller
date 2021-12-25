#!/usr/bin/python3

import os
import yaml
import time
import threading
import numpy as np
import RPi.GPIO as GPIO

from .constants import *
from . import wheels

def parse_yaml_config(configPath):
    # Validate the config file name
    # if configPath is None:
    #     raise Exception("Config file None.")

    if configPath is None:
        if os.path.exists('./config.yaml'):
            # print('yo')
            configPath = './config.yaml'
        else:
            # Place config in current directory
            print('Cannot find config file:', configPath)
            exit(1)
            # pass
    else:
        if os.path.exists(configPath):
            pass
        else:
            print('Cannot find config file:', configPath)
            exit(1)

    try:
        # Parse the YAML config. Expand any environment variables
        # inside, prior to parsing.
        with open(configPath, 'r') as configFile:
            config = yaml.safe_load(os.path.expandvars(configFile.read()))
    except IOError:
        raise Exception("Could not read config file {}.".format(configPath))

    # Validate the YAML specification
    try:
        if 'scuttle' not in config:
            raise Exception("'scuttle' section not present")
        elif 'chassis' not in config['scuttle']:
            raise Exception("'chassis' section not present")

        chassis = config['scuttle']['chassis']

        if 'wheels' not in chassis:
            raise Exception("'chassis' section not present")

        wheels = chassis['wheels']

        if 'l_wheel' not in wheels:
            raise Exception("'l_wheel' section not present")
        elif 'r_wheel' not in wheels:
            raise Exception("'r_wheel' section not present")

        l_wheel = wheels['l_wheel']
        r_wheel = wheels['r_wheel']

        if 'motor_control_gpio' not in l_wheel:
            raise Exception("'motor_control_gpio' section not present in'l_wheel'")
        elif 'motor_control_gpio' not in r_wheel:
            raise Exception("'r_wheel' section not present in'r_wheel'")
    except:
        raise

    return config

class SCUTTLE:

    def __init__(self, config=None, openLoop=True):

        GPIO.setmode(GPIO.BOARD)

        try:
            config = parse_yaml_config(config)
        except IOError:
            print("Config file parse failed.")
            raise

        # print(yaml.dump(config))

        self.heading = 0
        self.velocity = 0
        self.angularVelocity = 0
        self.globalPosition = [0, 0]

        chassisdB = config['scuttle']['chassis']
        wheelsdB  = chassisdB['wheels']

        l_wheel  = wheelsdB['l_wheel']
        l_gpio   = l_wheel['motor_control_gpio']
        l_motor_invert = l_wheel['motor_control_gpio']['invert']
        l_encoder_invert = l_wheel['encoder']['invert']
        l_encoder_address = l_wheel['encoder']['address']
        l_motor_channel   = (l_gpio['digital'], l_gpio['pwm'])

        r_wheel  = wheelsdB['r_wheel']
        r_gpio   = r_wheel['motor_control_gpio']
        r_motor_invert = r_wheel['motor_control_gpio']['invert']
        r_encoder_invert = r_wheel['encoder']['invert']
        r_encoder_address = r_wheel['encoder']['address']
        r_motor_channel  = (r_gpio['digital'], r_gpio['pwm'])

        # Get the wheel base from the config specification. If not
        # present then use the default value
        self.wheelBase = chassisdB.get('wheel_base', WHEEL_BASE)

        # Get the wheel radius from the config specification. If not
        # present then use the default value
        self.wheelRadius = chassisdB.get('wheel_radius', WHEEL_RADIUS)

        # Get the i2c bus id from the config specification. If not
        # present then use the default value
        i2c_bus = wheelsdB.get('i2c_bus_id', I2C_BUS)

        self.leftWheel          = wheels.Wheel(l_motor_channel,                             # Create left wheel object
                                               i2c_bus,
                                               l_encoder_address,
                                               invertEncoder=l_encoder_invert,
                                               invertMotor=l_motor_invert,
                                               openLoop=openLoop,
                                               )

        self.rightWheel         = wheels.Wheel(r_motor_channel,                            # Create right wheel object
                                               i2c_bus,
                                               r_encoder_address,
                                               invertEncoder=r_encoder_invert,
                                               invertMotor=r_motor_invert,
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
