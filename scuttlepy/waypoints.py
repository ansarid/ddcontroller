import time
from robot import SCUTTLE

# waypoints = [
#             [ 0.3,    0],
#             [ 0.3,  0.3],
#             [ 0.0,  0.3],
#             [   0,    0],
#             ]

# waypoints = [
#             [ 0.3, 0],
#             [ 0.3, 0.3],
#             [ 0.6, 0.3],
#             [ 0.6, 0.6],
#             [ 0.9, 0.6],
#             ]

waypoints = [
            [ 0.3, 0],
            [ 0.3, 0.5],
            [ 0.8, 0.5],
            [ 0.8, 0]
            ]

# waypoints = [
#             [ 0.3, 0],
#             [ 0.5, 0.5],
#             [ 1, 0.5],
#             ]

scuttle = SCUTTLE()

# print(dir(scuttle))

# scuttle.move([0.5, 0.5])

for waypoint in waypoints:
    print(waypoint)
    scuttle.move(waypoint)

# for waypoint in waypoints:
#     print("DRIVING TO POINT", waypoint)
#     scuttle.move(waypoint)
#     print("COMPLETED POINT", waypoint)
#     print("\n")

# startTime = time.time()

# try:
#     while 1:
#         if (time.time() - startTime) <= 5:
#             scuttle.setMotion([0.15, 0])
#         else:
#             scuttle.setMotion([0, 0])

# except:
#     pass

