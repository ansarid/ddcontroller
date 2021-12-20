import numpy as np
from scuttlepy.wheels import Wheel

leftWheel = Wheel((11,12), 0x40, invert_encoder=True)   # Left Motor  (ch1)
rightWheel = Wheel((15,16), 0x41) 	                    # Right Motor (ch2)

try:
    while True:

        leftWheel.setAngularVelocity(np.pi)
        rightWheel.setAngularVelocity(np.pi)

        print(round(leftWheel.getAngularVelocity(),3), ' rad/s\t', round(rightWheel.getAngularVelocity(), 3), ' rad/s')

except KeyboardInterrupt:

    print('Stopping')

finally:

    leftWheel.stop()
    rightWheel.stop()
