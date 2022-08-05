import time
from scuttlepy.encoder import Encoder

# Create encoder object
encoder = Encoder(0x40)

try:

    # Create infinite loop
    while True:

        # Read the encoder position
        position = encoder.read_position()

        # Read the encoder angle
        angle = encoder.read_angle()

        # Print out the encoder position and angle
        print("Angle: {}\tPosition: {}".format(round(angle, 3), position))

        # Run loop at 50Hz
        time.sleep(1/50)

except KeyboardInterrupt:
    pass

finally:
    pass