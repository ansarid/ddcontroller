#!/usr/bin/python3

# This example drives the right and left motors.
# Intended for Beaglebone Blue hardware.
# This example uses rcpy library. Documentation: guitar.ucsd.edu/rcpy/rcpy.pdf

from adafruit_platformdetect import Detector
detector = Detector()

# Import external libraries

class Motor:

    def __init__(self, channel=None, pins=None, invert=False):

        self.duty = 0                                   # Initial Duty
        self.invert = invert                            # Reverse motor direction? Duty of 1 becomes -1 and duty of -1 becomes 1

        if detector.board.BEAGLEBONE_BLUE:
            import rcpy
            import rcpy.motor as motor

            if isinstance(channel, int):
                if 1 >= channel <= 4:
                    self.channel = channel
                    rcpy.set_state(rcpy.RUNNING)
                    motor.set(self.channel, self.duty)
                else:
                    raise ValueError('Invalid Channel. Please use a channel from 1-4.')
            else:
                raise TypeError('channel must be an int.')

        if detector.board.any_raspberry_pi_40_pin:
            import RPi.GPIO as GPIO
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BOARD)

            if isinstance(pins, tuple):
                self.pins = pins

                for pin in self.pins:
                    GPIO.setup(pin, GPIO.OUT)
                    GPIO.output(pin, GPIO.LOW)

                self.pins = (GPIO.PWM(self.pins[0], 25000),
                             GPIO.PWM(self.pins[1], 25000)
                            )

                for pin in self.pins:
                    pin.start(0)
                    pin.ChangeDutyCycle(100)     # Change duty cycle


            else:
                raise TypeError('pins must be a tuple.')

    def setDuty(self, duty):
        if detector.board.BEAGLEBONE_BLUE:
            if rcpy.get_state() == rcpy.RUNNING:            # Execute loop if rcpy is running
                if not self.invert:
                    self.duty = duty
                else:
                    self.duty = -1 * duty                   # Invert duty cycle

                motor.set(self.channel, self.duty)

        if detector.board.any_raspberry_pi_40_pin:
            if not self.invert:
                if duty > 0:
                    self.pins[0].ChangeDutyCycle(0)     # Change duty cycle
                    self.pins[1].ChangeDutyCycle(abs(duty)*100)     # Change duty cycle
                if duty < 0:
                    self.pins[0].ChangeDutyCycle(abs(duty)*100)     # Change duty cycle
                    self.pins[1].ChangeDutyCycle(0)     # Change duty cycle
            else:
                if duty > 0:
                    self.pins[0].ChangeDutyCycle(0)     # Change duty cycle
                    self.pins[1].ChangeDutyCycle(abs(duty)*100)     # Change duty cycle
                if duty < 0:
                    self.pins[0].ChangeDutyCycle(abs(duty)*100)     # Change duty cycle
                    self.pins[1].ChangeDutyCycle(0)     # Change duty cycle

if __name__ == "__main__":

    import time

    if detector.board.BEAGLEBONE_BLUE:
        l_motor = Motor(channel=1) 	                                # Create Left Motor Object (ch1)
        r_motor = Motor(channel=2) 	                                # Create Right Motor Object (ch2)

    if detector.board.any_raspberry_pi_40_pin:
        r_motor = Motor(pins=(11,12)) 	                                # Create Left Motor Object (ch1)
        l_motor = Motor(pins=(15,16)) 	                                # Create Right Motor Object (ch2)

    while True:
        print("motors.py: driving fwd")
        l_motor.setDuty(1)                         # Set left motor duty cycle to 0.7
        r_motor.setDuty(1)                          # Set right motor duty cycle to 0.7
        time.sleep(5)                              # Wait 5 seconds
        print("motors.py: driving reverse")
        l_motor.setDuty(-1)                          # Set left motor duty cycle to -0.7
        r_motor.setDuty(-1)                         # Set right motor duty cycle to -0.7
        time.sleep(5)                              # Wait 5 seconds
