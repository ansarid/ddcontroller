#!/usr/bin/python3

from adafruit_platformdetect import Detector
detector = Detector()

if detector.board.BEAGLEBONE_BLUE:

    import rcpy
    import rcpy.motor as motor

    class Motor:

        def __init__(self, channel, invert=False):

            self.channel = channel
            self.duty = 0                                   # Initial Duty
            self.invert = invert                            # Reverse motor direction? Duty of 1 becomes -1 and duty of -1 becomes 1

            rcpy.set_state(rcpy.RUNNING)
            motor.set(self.channel, self.duty)

        def setDuty(self, duty):
            if rcpy.get_state() == rcpy.RUNNING:            # Execute loop if rcpy is running
                self.duty = round(sorted((-1, duty, 1))[1], 2)
                if not self.invert:
                    self.duty = duty
                else:
                    self.duty = -1 * duty                   # Invert duty cycle

                motor.set(self.channel, self.duty)

        def stop(self):
            motor.set(self.channel, 0)
            rcpy.set_state(rcpy.EXITING)


elif detector.board.any_raspberry_pi_40_pin or detector.board.JETSON_NANO:

    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)

    class Motor:

        def __init__(self, pins, freqency=150, invert=False):

            self.pins = pins                                # First pin will be PWM and second pin will be digital
            self.duty = 0                                   # Initial Duty
            self.freqency = freqency                        # PWM freqency (Hz)
            self.invert = invert                            # Reverse motor direction? Duty of 1 becomes -1 and duty of -1 becomes 1

            for pin in pins:
                GPIO.setup(pin, GPIO.OUT)

            self.motor = GPIO.PWM(pins[0], self.freqency)
            self.motor.start(self.duty)

        def setDuty(self, duty):

            self.duty = round(sorted((-1, duty, 1))[1], 2)

            if self.duty > 0:
                GPIO.output(self.pins[1], self.duty < 0)
                self.motor.ChangeDutyCycle(abs(self.duty*100))

            elif self.duty < 0:
                GPIO.output(self.pins[1], self.duty < 0)
                self.motor.ChangeDutyCycle(100-abs(self.duty*100))

            elif self.duty == 0:
                GPIO.output(self.pins[1], False)
                self.motor.ChangeDutyCycle(0)

        def stop(self):
            GPIO.output(self.pins[1], False)
            self.motor.stop()
            GPIO.cleanup(self.pins)


if __name__ == "__main__":

    import time
    import numpy as np

    if detector.board.BEAGLEBONE_BLUE:

        l_motor = Motor(1) 	                            # Create Left Motor Object (ch1)
        r_motor = Motor(2) 	                            # Create Right Motor Object (ch2)

    elif detector.board.any_raspberry_pi_40_pin:

        l_motor = Motor((15,16)) 	                    # Create Left Motor Object (pwm, digital)
        r_motor = Motor((11,12)) 	                    # Create Right Motor Object (pwm, digital)

    elif detector.board.JETSON_NANO:

        l_motor = Motor((32,29))                        # Create Left Motor Object (pwm, digital)
        r_motor = Motor((33,31))                        # Create Right Motor Object (pwm, digital)

    else:
        print('Unsupported Platform "', detector.board.id, '"!')
        exit()

    try:
        while True:

            for duty in np.arange(-1,1, 0.01):
                print(duty)
                l_motor.setDuty(duty)           # Set left motor duty cycle
                r_motor.setDuty(duty)           # Set right motor duty cycle
                time.sleep(0.1)                 # Wait 0.1 seconds

            for duty in np.arange(1,-1, -0.01):
                print(duty)
                l_motor.setDuty(duty)           # Set left motor duty cycle
                r_motor.setDuty(duty)           # Set right motor duty cycle
                time.sleep(0.1)                 # Wait 0.1 seconds

    finally:

        l_motor.stop()
        r_motor.stop()
