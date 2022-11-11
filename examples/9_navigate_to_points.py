import time
from ddcontroller import DDRobot

robot = DDRobot(debug=True)

points = [
        [1,0],
        [1,1],
        [0,1],
        [0,0],
        ]

try:
    for point in points:
        robot.go_to(point, tolerance=0.01, max_linear_velocity=0.3, max_angular_velocity=1.5, backwards=False)

        while robot.running and not robot.reached_target_position:
            x, y = robot.get_global_position()
            print(f"Global Position: {round(x, 3)}, {round(y, 3)}", end="\r")
            time.sleep(1/50)
        print('\nDone!')

except KeyboardInterrupt:
    print('Stopping...')

finally:
    # Clean up.
    robot.stop()
    print('Stopped.')