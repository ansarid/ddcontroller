import numpy as np
from scuttlepy.wheels import Wheel

if __name__ == "__main__":

    from adafruit_platformdetect import Detector
    detector = Detector()

    if detector.board.BEAGLEBONE_BLUE:

        l_wheel = Wheel(1, 0x40, invert_encoder=True)           # Left Motor  (ch1)
        r_wheel = Wheel(2, 0x41) 	                            # Right Motor (ch2)

    elif detector.board.any_raspberry_pi_40_pin:

        l_wheel = Wheel((15,16), 0x43, invert_encoder=True)     # Left Motor  (ch1)
        r_wheel = Wheel((11,12), 0x41) 	                        # Right Motor (ch2)

    elif detector.board.JETSON_NANO:

        l_wheel = Wheel((32,29), 0x43, invert_encoder=True)     # Left Motor  (ch1)
        r_wheel = Wheel((33,31), 0x41) 	                        # Right Motor (ch2)

    else:
        print('Unsupported Platform "', detector.board.id, '"!')
        exit()

    try:

        while True:

            r_wheel.setAngularVelocity(np.pi)
            l_wheel.setAngularVelocity(np.pi)

            print(l_wheel.getAngularVelocity(), ' rad/s\t', r_wheel.getAngularVelocity(), ' rad/s')

    except KeyboardInterrupt:
        print('Stopping')

    finally:
        r_wheel.stop()
        l_wheel.stop()
