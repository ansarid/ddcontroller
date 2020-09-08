import time
from robot import SCUTTLE

waypoints = [
            [ 0.3, 0],
            [ 0.3, 0.6],
            [ 0.9, 0.6],
            [ 0.9, 0]
            ]


waypoints = [[2,0]]

scuttle = SCUTTLE()

scuttle.setup() # take the first encoder readings, establish start

for waypoint in waypoints:
    print(waypoint)
    scuttle.move(waypoint)

# startTime = time.time()

# try:
#     while 1:
#         if (time.time() - startTime) <= 5:
#             scuttle.setMotion([0.15, 0])
#         else:
#             scuttle.setMotion([0, 0])

# except:
#     pass

