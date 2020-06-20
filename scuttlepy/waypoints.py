import time
from robot import SCUTTLE

# waypoints = [
#             [0.3,     0],
#             [ 0.3,  0.3],
#             [ 0.0,  0.3],
#             [0,       0],
#             ]

# waypoints = [
#             [0, 0.3],
#             ]

waypoints = [
            [ 0.3, 0],
            [ 0.3, 0.3],
            [ 0.6, 0.3],
            [ 0.6, 0.6],
            [ 0.9, 0.6],
            ]


scuttle = SCUTTLE()

for waypoint in waypoints:
    print("DRIVING TO POINT", waypoint)
    scuttle.move(waypoint)
    print("COMPLETED POINT", waypoint)
    print("\n")

# startTime = time.time()

# try:
#     while 1:
#         if (time.time() - startTime) <= 5:
#             scuttle.setMotion([0.15, 0])
#         else:
#             scuttle.setMotion([0, 0])

# except:
#     pass

