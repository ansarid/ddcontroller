#!/usr/bin/python3

import time
from scuttlepy.motor import Motor

leftMotor = Motor((11,12)) 	                # Create Left Motor Object (pwm, digital)
rightMotor = Motor((15,16)) 	                # Create Right Motor Object (pwm, digital)

try:

    while True:

        for duty in range(100, -100, -1):
            print(duty)
            leftMotor.setDuty(duty/100)           # Set left motor duty cycle
            rightMotor.setDuty(duty/-100)           # Set right motor duty cycle
            time.sleep(0.05)                    # Wait 0.1 seconds

        for duty in range(-100, 100,  1):
            print(duty)
            leftMotor.setDuty(duty/100)           # Set left motor duty cycle
            rightMotor.setDuty(duty/-100)           # Set right motor duty cycle
            time.sleep(0.05)                    # Wait 0.1 secon

except KeyboardInterrupt:

    print('Stopping...')

finally:

    leftMotor.stop()
    rightMotor.stop()
    print('Stopped.')
