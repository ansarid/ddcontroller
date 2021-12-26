#!/usr/bin/python3

import numpy as np
from scuttlepy import SCUTTLE
from scuttlepy.constants import *

scuttle = SCUTTLE()

try:

    print(scuttle.getGlobalPosition())
    scuttle.move(0.25)
    scuttle.turn(90)
    print(np.degrees(scuttle.getHeading()))
    scuttle.move(0.25)
    scuttle.turn(90)
    print(np.degrees(scuttle.getHeading()))
    scuttle.move(0.25)
    scuttle.turn(90)
    print(np.degrees(scuttle.getHeading()))
    scuttle.move(0.25)
    scuttle.turn(90)
    print(np.degrees(scuttle.getHeading()))
    print(scuttle.getGlobalPosition())

except KeyboardInterrupt:

    pass

finally:

    scuttle.stop()
