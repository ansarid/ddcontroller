#!/usr/bin/python3

import time
import numpy as np
from scuttlepy.constants import *
from scuttlepy.wheels import Wheel

settings = Settings(file=config)

leftWheel = Wheel((11,12), 1,0x40, invertEncoder=True)	    # Create Left Wheel Object
rightWheel = Wheel((15,16), 1,0x41) 	               	    # Create Right Wheel Object

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
