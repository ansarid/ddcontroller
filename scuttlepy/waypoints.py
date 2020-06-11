from robot import SCUTTLE

waypoints = [
            [   0,  0.2],
            [ 0.2,  0.2],
            [ 0.2, -0.2],
            [-0.2, -0.2],
            [-0.2,  0.2],
            [   0,  0.2],
            [   0,    0],
            ]

scuttle = SCUTTLE()

for waypoint in waypoints:
    print("DRIVING TO POINT", waypoint)
    scuttle.move(waypoint)
    print("COMPLETED POINT", waypoint)
    print("\n")
