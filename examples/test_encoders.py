import time
from scuttlepy.encoder import Encoder

if __name__ == "__main__":

    leftEncoder = Encoder(0x40)				# Create encoder object for left encoder on address 0x41
    rightEncoder = Encoder(0x41, invert=True)           # Create encoder object for right encoder on address 0x40

    try:

        while True:

            leftAngle = round(leftEncoder.readAngle(), 2)
            rightAngle = round(rightEncoder.readAngle(), 2)

            print(leftAngle, "\t", rightAngle)

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass

    finally:
        pass
