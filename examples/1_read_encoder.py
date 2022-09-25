import time
from as5048b import AS5048B

# Create encoder object
encoder = AS5048B(0x40)

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