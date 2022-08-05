import math
import time
from scuttlepy import SCUTTLE

# Create SCUTTLE object
scuttle = SCUTTLE()

try:

    # Create infinite loop
    while True:

        # Set SCUTTLE's heading to pi/2
        scuttle.setHeading(math.pi/2)

        # Print the motion of the SCUTTLE
        print(scuttle.getHeading())

        # Run loop at 50Hz
        time.sleep(1/50)

except KeyboardInterrupt:
    print('Stopping...')

finally:
    # Clean up.
    scuttle.stop()
    print('Stopped.')