import time
from gamepad import Gamepad
from scuttlepy import SCUTTLE

gamepad = Gamepad()
scuttle = SCUTTLE()

try:

    while True:

        joystickX, joystickY = [((-2/255)*gamepad.axes['LEFT_X'])+1,    # Get joystick x and y values and scale values to -1 to 1
                                ((-2/255)*gamepad.axes['LEFT_Y'])+1
                                ]

        motion = [joystickY*scuttle.maxVelocity,            # Mutiply joystick y axis by scuttle max linear velocity to get linear velocity
                  joystickX*scuttle.maxAngularVelocity      # Mutiply joystick x axis by scuttle max angular velocity to get angular velocity
                 ]

        print(scuttle.getGlobalPosition())
        scuttle.setMotion(motion)               # Set motion to SCUTTLE

        time.sleep(0.05)

except KeyboardInterrupt:

    pass

finally:

    gamepad.close()
    scuttle.stop()