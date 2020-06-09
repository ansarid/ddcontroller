from scuttlepy import robot

waypoints = [
            [1, 1],
            [1, 1],
            [1, 1],
            [1, 1],
            [1, 1],
            ]

scuttle = robot.SCUTTLE()

for waypoint in waypoints:
    scuttle.move(waypoint)
