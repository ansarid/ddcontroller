#!/usr/bin/python3

from scuttlepy import SCUTTLE
from scuttlepy.constants import *

scuttle = SCUTTLE()

try:

    while True:

        scuttle.setMotion([0.4, 0])
        print(scuttle.getGlobalPosition())

except KeyboardInterrupt:

    pass

finally:

    scuttle.stop()
