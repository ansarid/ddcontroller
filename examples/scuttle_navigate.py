#!/usr/bin/python3

from scuttlepy import SCUTTLE
from scuttlepy.constants import *

scuttle = SCUTTLE()

waypoints = [(1,0),     # Waypoints to move in 1x1m square path
             (1,1),
             (0,1),
             (0,0),
            ]

try:

    for waypoint in waypoints:  # Navigate to each waypoint in list of waypoints
        scuttle.goTo(waypoint)

except KeyboardInterrupt:

    pass

finally:

    scuttle.stop()
