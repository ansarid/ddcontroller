import time
from scuttlepy import SCUTTLE

# Create SCUTTLE object
scuttle = SCUTTLE()

try:

    # Set target location for navigation to (1,1)
    scuttle.goTo([1,1])

    # Loop while SCUTTLE is in motion
    while scuttle.isMoving():

        # Get the SCUTTLE's latest location
        x,y = scuttle.getGlobalPosition()

        # Print the location of the SCUTTLE
        print('Global Position: {}, {}'.format(round(x, 3), round(y, 3)))

        # Run loop at 50Hz
        time.sleep(1/50)

except KeyboardInterrupt:
    print('Stopping...')

finally:
    # Clean up.
    scuttle.stop()
    print('Stopped.')