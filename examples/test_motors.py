#!/usr/bin/python3

from scuttlepy.motor import Motor

from adafruit_platformdetect import Detector
detector = Detector()

if __name__ == "__main__":

    import time

    if detector.board.BEAGLEBONE_BLUE:

        leftMotor = Motor(1) 	                        # Create Left Motor Object (ch1)
        rightMotor = Motor(2) 	                        # Create Right Motor Object (ch2)

    elif detector.board.any_raspberry_pi_40_pin:

        leftMotor = Motor((11,12)) 	                # Create Left Motor Object (pwm, digital)
        rightMotor = Motor((15,16)) 	                # Create Right Motor Object (pwm, digital)

    elif detector.board.JETSON_NANO:

        leftMotor = Motor((32,29))                    # Create Left Motor Object (pwm, digital)
        rightMotor = Motor((33,31))                    # Create Right Motor Object (pwm, digital)

    else:
        print('Unsupported Platform "', detector.board.id, '"!')
        exit()

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
        print('Stopping')

    finally:

        leftMotor.stop()
        rightMotor.stop()
