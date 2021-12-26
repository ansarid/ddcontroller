import time
from gamepad import Gamepad
from scuttlepy import SCUTTLE

gamepad = Gamepad()
scuttle = SCUTTLE()

waypoints = []
waypoints.append(scuttle.getGlobalPosition())
waypointsPos = 0

try:

    while True:

        joystickX, joystickY = [((-2/255)*gamepad.axes['LEFT_X'])+1,    # Get joystick x and y values and scale values to -1 to 1
                                ((-2/255)*gamepad.axes['LEFT_Y'])+1
                                ]

        motion = [joystickY*scuttle.maxVelocity,            # Mutiply joystick y axis by scuttle max linear velocity to get linear velocity
                  joystickX*scuttle.maxAngularVelocity      # Mutiply joystick x axis by scuttle max angular velocity to get angular velocity
                 ]

        # print(scuttle.getGlobalPosition())

        if gamepad.buttons['X']:                            # Save Waypoint
            waypoints.append(scuttle.getGlobalPosition())
            waypointsPos += 1
            print('Added waypoint', waypoints[waypointsPos], 'to waypoints',waypoints)

        elif gamepad.buttons['BACK']:                       # Go to Previous Waypoint
            if waypointsPos > 0:
                waypointsPos -= 1
                print('Going to previous waypoint:', waypoints[waypointsPos])
                scuttle.goTo(waypoints[waypointsPos])
            else:
                print('No previous waypoint set.')

        elif gamepad.buttons['START']:                      # Go to Global (0,0)
            print('Going to Start:', waypoints[0])
            scuttle.goTo(waypoints[0])

        else:
            scuttle.setMotion(motion)                       # Set motion to SCUTTLE

        time.sleep(0.05)

except KeyboardInterrupt:
    gamepad.close()
    scuttle.stop()