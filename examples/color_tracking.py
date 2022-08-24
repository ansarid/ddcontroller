#!/usr/bin/python3

"""
This file is part of the robotPy library (https://github.com/ansarid/ddcontroller).
Copyright (C) 2022  Daniyal Ansari

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

import cv2
import time
import numpy as np
from ddcontroller import DDRobot

# Color Range, described in HSV

v1_min = 0  # Minimum H value
v2_min = 140  # Minimum S value
v3_min = 50  # Minimum V value

v1_max = 20  # Maximum H value
v2_max = 220  # Maximum S value
v3_max = 230  # Maximum V value

min_radius = 6  # Minimum radius of object for tracking
target_radius = 60  # Target radius for object
deadband = (
    0.3  # Angular velocity range to mute to remove oscillations when object is centered
)

image_resize = (240, 160)  # Reduce image size to speed up processing

robot = DDRobot()  # Create robot object

linearVelocityMultiplier = 1
# robot.maxLinearVelocity = 0.3 # Set the maximum linear velocity
robot.maxAngularVelocity = 1.5  # Set the angular linear velocity

loop_freq = 15


def slope_intercept(x1, y1, x2, y2):
    a = (y2 - y1) / (x2 - x1)
    b = y1 - a * x1
    return a, b


def rotateImage(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result


def sleep(target_freq, startTime):
    # Measure time since start and subtract from sleep time
    time.sleep(sorted([(1 / target_freq) - ((time.monotonic() - startTime)), 0])[1])


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
            image = cv2.resize(image, image_resize, interpolation=cv2.INTER_LINEAR)

            # The input image is in BGR format. COnvert to HSV
            frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Apply the threshold to filter the image. The pixels outside the range will be black
            # after this operation.
            thresh = cv2.inRange(
                frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max)
            )

            # Perform guassian blur. The following does the Dilation followed by Erosion.
            # It closes any holes inside the object.
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

            # Find all contours
            contours = cv2.findContours(
                mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )[-2]

            # If more than 0 objects found
            if len(contours) > 0:

                # Get the contour with max area
                c = max(contours, key=cv2.contourArea)

                # Put a circle around the contour of interest
                ((x, y), radius) = cv2.minEnclosingCircle(c)

                # If object is larger than minimum tracking radius
                if radius > min_radius:

                    # Calculate angular velocity
                    a, b = slope_intercept(
                        0,
                        robot.maxAngularVelocity,
                        image_resize[0],
                        -robot.maxAngularVelocity,
                    )
                    angular_velocity = (a * x) + b

                    # Calculate linear velocity
                    a, b = slope_intercept(
                        image_resize[1], -robot.maxVelocity, target_radius, 0
                    )
                    linear_velocity = (
                        (a * (radius * 2)) + b
                    ) * linearVelocityMultiplier

                    # Create deadband to remove oscillations
                    # if angular velocity is less than specified range
                    if abs(angular_velocity) < deadband:

                        # Set angular velocity to 0
                        angular_velocity = 0

                    # Set robot linear and angular velocity
                    robot.setMotion(
                        [round(linear_velocity, 2), round(angular_velocity, 2)]
                    )

            else:
                # Set robot linear velocity to 0
                robot.setLinearVelocity(0)

        #         sleep(loop_freq,startTime)
        print(round(1 / (time.monotonic() - startTime), 2), "Hz", end="\r")

except KeyboardInterrupt:

    pass

finally:
    # Clean up.
    camera.release()
    robot.stop()
    print("Stopped.")
