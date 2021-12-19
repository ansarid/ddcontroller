import numpy as np
from scuttlepy.wheels import Wheel

if __name__ == "__main__":

    from adafruit_platformdetect import Detector
    detector = Detector()

    if detector.board.BEAGLEBONE_BLUE:

        leftWheel = Wheel(1, 0x40, invert_encoder=True)           # Left Motor  (ch1)
        rightWheel = Wheel(2, 0x41) 	                            # Right Motor (ch2)

    elif detector.board.any_raspberry_pi_40_pin:

        leftWheel = Wheel((11,12), 0x40, invert_encoder=True)     # Left Motor  (ch1)
        rightWheel = Wheel((15,16), 0x41) 	                        # Right Motor (ch2)

    elif detector.board.JETSON_NANO:

        leftWheel = Wheel((32,29), 0x40, invert_encoder=True)     # Left Motor  (ch1)
        rightWheel = Wheel((33,31), 0x41) 	                        # Right Motor (ch2)

    else:
        print('Unsupported Platform "', detector.board.id, '"!')
        exit()

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
