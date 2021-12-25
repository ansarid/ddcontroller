#!/usr/bin/python3

from scuttlepy import SCUTTLE
from scuttlepy.constants import *

scuttle = SCUTTLE()

try:

    while True:

        scuttle.setMotion([100,0])
        #print(scuttle.getGlobalPosition())
        print(scuttle.getMotion())

except KeyboardInterrupt:

    pass

finally:

    scuttle.stop()
