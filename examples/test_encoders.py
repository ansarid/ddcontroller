import time
from scuttlepy.encoder import Encoder

if __name__ == "__main__":

    rightEncoder = Encoder(0x43, invert=True)                                  # Create encoder object for left encoder on address 0x40
    leftEncoder = Encoder(0x41)                                               # Create encoder object for right encoder on address 0x41

    try:

        while True:

            rightAngle = round(rightEncoder.readAngle(), 2)
            leftAngle = round(leftEncoder.readAngle(), 2)

            print(leftAngle, "\t", rightAngle)

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass

    finally:
        pass