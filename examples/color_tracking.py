#/usr/bin/python3

import cv2
import time
import numpy as np
from scuttlepy import SCUTTLE

# Color Range, described in HSV

v1_min = 0      # Minimum H value
v2_min = 140    # Minimum S value
v3_min = 50     # Minimum V value

v1_max = 20     # Maximum H value
v2_max = 220    # Maximum S value
v3_max = 230    # Maximum V value

min_radius = 6              # Minimum radius of object for tracking
target_radius = 60          # Target radius for object
deadband = 0.3              # Angular velocity range to mute to remove oscillations when object is centered

image_resize = (240,160)    # Reduce image size to speed up processing

scuttle = SCUTTLE()         # Create SCUTTLE object

linearVelocityMultiplier = 1
# scuttle.maxLinearVelocity = 0.3 # Set the maximum linear velocity
scuttle.maxAngularVelocity = 1.5  # Set the angular linear velocity

loop_freq = 15

def slope_intercept(x1,y1,x2,y2):
    a = (y2 - y1) / (x2 - x1)
    b = y1 - a * x1
    return a,b

def rotateImage(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result

def sleep(target_freq, startTime):
    time.sleep(sorted([(1/target_freq)-((time.monotonic()-startTime)), 0])[1])    # Measure time since start and subtract from sleep time

# Create video capture object with camera at index 0
camera = cv2.VideoCapture(0)

# Check if camera was opened successfully
if camera.isOpened() == False:
    print("Error opening camera.")
    exit()

try:

    # Loop while camera is open
    while camera.isOpened():

        startTime = time.monotonic()

        # Read image from camera
        ret, image = camera.read()

        # If image capture read was successful
        if ret:

            # Resize the image to reduce processing overhead
            image = cv2.resize(image, image_resize, interpolation= cv2.INTER_LINEAR)

            # The input image is in BGR format. COnvert to HSV
            frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Apply the threshold to filter the image. The pixels outside the range will be black
            # after this operation.
            thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

            # Perform guassian blur. The following does the Dilation followed by Erosion.
            # It closes any holes inside the object.
            kernel = np.ones((5,5),np.uint8)
            mask = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

            # Find all contours
            contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]

            # If more than 0 objects found
            if len(contours) > 0:

                # Get the contour with max area
                c = max(contours, key=cv2.contourArea)

                # Put a circle around the contour of interest
                ((x, y), radius) = cv2.minEnclosingCircle(c)

                # If object is larger than minimum tracking radius
                if radius > min_radius:

                    # Calculate angular velocity
                    a, b = slope_intercept(0, scuttle.maxAngularVelocity, image_resize[0], -scuttle.maxAngularVelocity)
                    angular_velocity = (a*x)+b

                    # Calculate linear velocity
                    a, b = slope_intercept(image_resize[1], -scuttle.maxVelocity, target_radius, 0)
                    linear_velocity = ((a*(radius*2))+b)*linearVelocityMultiplier

                    # Create deadband to remove oscillations
                    # if angular velocity is less than specified range
                    if abs(angular_velocity) < deadband:

                        # Set angular velocity to 0
                        angular_velocity = 0

                    # Set SCUTTLE linear and angular velocity
                    scuttle.setMotion([round(linear_velocity, 2), round(angular_velocity, 2)])

            else:
                # Set SCUTTLE linear velocity to 0
                scuttle.setLinearVelocity(0)

#         sleep(loop_freq,startTime)
        print(round(1/(time.monotonic()-startTime),2), 'Hz', end='\r')

except KeyboardInterrupt:

    pass

finally:
    # Clean up.
    camera.release()
    scuttle.stop()
    print('Stopped.')
