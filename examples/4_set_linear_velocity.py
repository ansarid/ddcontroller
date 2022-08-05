import time
from scuttlepy import SCUTTLE

# Create SCUTTLE object
scuttle = SCUTTLE()

try:

    # Create infinite loop
    while True:

        # Set SCUTTLE's linear velocity to 0.2 m/s
        scuttle.setLinearVelocity(0.2)

        # Print the motion of the SCUTTLE
        print(scuttle.getMotion())

        # Run loop at 50Hz
        time.sleep(1/50)

except KeyboardInterrupt:
    print('Stopping...')

finally:
    # Clean up.
    scuttle.stop()
    print('Stopped.')