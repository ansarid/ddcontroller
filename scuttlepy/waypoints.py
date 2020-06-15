from robot import SCUTTLE

# waypoints = [
#             [0.3,     0],
#             [ 0.3,  0.3],
#             [ 0.0,  0.3],
#             [0,       0],
#             ]

waypoints = [
            [1, 0],
            ]

scuttle = SCUTTLE()

for waypoint in waypoints:
    print("DRIVING TO POINT", waypoint)
    scuttle.move(waypoint)
    print("COMPLETED POINT", waypoint)
    print("\n")
