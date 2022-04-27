#!/usr/bin/python3

import RPi.GPIO as GPIO

if GPIO.getmode() is None:
    GPIO.setmode(GPIO.BOARD)


class Motor:
    def __init__(self, pins, frequency=150, invert=False):
        """_summary_

        Args:
            pins (_type_): _description_
            frequency (int, optional): _description_. Defaults to 150.
            invert (bool, optional): _description_. Defaults to False.
        """
        # First pin will be digital and second pin will be PWM
        self.pins = pins

        # Initial Duty %
        self.duty = 0

        # PWM frequency (Hz)
        self.frequency = frequency

        # Reverse motor direction
        self.invert = invert

        for pin in pins:  # Set motor pins as outputs
            GPIO.setup(pin, GPIO.OUT)

        self.motor = GPIO.PWM(  # set first pin as PWM and set freq
            pins[1], self.frequency
        )
        self.motor.start(self.duty)

    def set_duty(self, duty):
        """_summary_

        Args:
            duty (_type_): _description_
        """
        self.duty = round(  # Make sure duty is between -1 and 1
            sorted((-1, float(duty), 1))[1], 2
        )

        duty = self.duty * 100

        GPIO.output(  # Set direction pin high if duty cycle is negative
            self.pins[0], duty < 0
        )

        if duty == 0:
            GPIO.output(self.pins[0], False)
            self.motor.ChangeDutyCycle(0)

        else:
            self.motor.ChangeDutyCycle(duty if (duty > 0) else abs(100 + duty))

    def stop(self):
        """_summary_"""
        GPIO.output(self.pins[0], False)
        self.motor.stop()
        GPIO.cleanup(self.pins)
