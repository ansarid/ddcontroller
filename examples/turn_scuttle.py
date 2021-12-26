#!/usr/bin/python3

import time
import numpy as np
from scuttlepy import SCUTTLE
from scuttlepy.constants import *

scuttle = SCUTTLE()

try:

    print(np.degrees(scuttle.getHeading()))
    # scuttle.turn(90, 2)                         # Turn 90 degrees at 2 rad/s
    scuttle.turnPID(-90)                        # Turn 90 degrees using PID
    print(np.degrees(scuttle.getHeading()))

except KeyboardInterrupt:

    pass

finally:

    scuttle.stop()
