#!/usr/bin/python3

from scuttlepy.motor import Motor

from adafruit_platformdetect import Detector
detector = Detector()

if __name__ == "__main__":

    import time

    if detector.board.BEAGLEBONE_BLUE:

        l_motor = Motor(1) 	                        # Create Left Motor Object (ch1)
        r_motor = Motor(2) 	                        # Create Right Motor Object (ch2)

    elif detector.board.any_raspberry_pi_40_pin:

        l_motor = Motor((15,16)) 	                # Create Left Motor Object (pwm, digital)
        r_motor = Motor((11,12)) 	                # Create Right Motor Object (pwm, digital)

    elif detector.board.JETSON_NANO:

        l_motor = Motor((32,29))                    # Create Left Motor Object (pwm, digital)
        r_motor = Motor((33,31))                    # Create Right Motor Object (pwm, digital)

    else:
        print('Unsupported Platform "', detector.board.id, '"!')
        exit()

    try:
        while True:

            for duty in range(100, -100, -1):
                print(duty)
                l_motor.setDuty(duty/100)           # Set left motor duty cycle
                r_motor.setDuty(duty/100)           # Set right motor duty cycle
                time.sleep(0.05)                    # Wait 0.1 seconds

            for duty in range(-100, 100,  1):
                print(duty)
                l_motor.setDuty(duty/100)           # Set left motor duty cycle
                r_motor.setDuty(duty/100)           # Set right motor duty cycle
                time.sleep(0.05)                    # Wait 0.1 secon

    except KeyboardInterrupt:
        print('Stopping')

    finally:

        l_motor.stop()
        r_motor.stop()
