import time
from scuttlepy.constants import *
from scuttlepy.encoder import Encoder

settings = Settings(file=config)

if __name__ == "__main__":

    leftEncoder = Encoder(settings.LEFT_WHEEL_ENCODER_ADDRESS,      # Create encoder object for left encoder
                          settings.LEFT_WHEEL_ENCODER_INVERT
                          )

    rightEncoder = Encoder(settings.RIGHT_WHEEL_ENCODER_ADDRESS,     # Create encoder object for right encoder
                          settings.RIGHT_WHEEL_ENCODER_INVERT
                          )

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
