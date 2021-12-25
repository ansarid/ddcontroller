#!/usr/bin/python3

import time
import numpy as np
from scuttlepy.constants import *
from scuttlepy.wheels import Wheel

settings = Settings()


leftWheel  = Wheel(settings.LEFT_WHEEL_MOTOR_PINS,                             # Create left wheel object
                   settings.I2C_BUS,
                   settings.LEFT_WHEEL_ENCODER_ADDRESS,
                   invertEncoder=settings.LEFT_WHEEL_ENCODER_INVERT,
                   invertMotor=settings.LEFT_WHEEL_MOTOR_INVERT,
                   openLoop=settings.OPENLOOP,
                   )

rightWheel = Wheel(settings.RIGHT_WHEEL_MOTOR_PINS,                            # Create right wheel object
                   settings.I2C_BUS,
                   settings.RIGHT_WHEEL_ENCODER_ADDRESS,
                   invertEncoder=settings.RIGHT_WHEEL_ENCODER_INVERT,
                   invertMotor=settings.RIGHT_WHEEL_MOTOR_INVERT,
                   openLoop=settings.OPENLOOP,
                   )

try:

    while True:

        leftWheel.update()
        rightWheel.update()

        leftWheel.setAngularVelocity(2*np.pi)
        rightWheel.setAngularVelocity(2*np.pi)

        print(round(leftWheel.getAngularVelocity(),3), ' rad/s\t', round(rightWheel.getAngularVelocity(), 3), ' rad/s')

        time.sleep(1/50)

except KeyboardInterrupt:

    print('Stopping...')

finally:

    leftWheel.stop()
    rightWheel.stop()
    print('Stopped.')
