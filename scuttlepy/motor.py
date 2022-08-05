#!/usr/bin/python3

'''
This file is part of the SCUTTLEPy library (https://github.com/ansarid/scuttlepy).
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
'''


import RPi.GPIO as GPIO
GPIO.setwarnings(False)

class Motor:
    """_summary_
        Motor
    """

    def __init__(self, digital_pin, pwm_pin, pwm_frequency, invert=False):
        """_summary_

        Args:
            digital_pin (int, optional): _description_.
            pwm_pin (int, optional): _description_.
            pwm_frequency (int, optional): _description_.
            invert (bool, optional): _description_. Defaults to False.
        """

        self.digital_pin = digital_pin
        self.pwm_pin = pwm_pin

        # First pin will be digital and second pin will be PWM
        self.pins = (self.digital_pin, self.pwm_pin)

        # Initial Duty %
        self.duty = 0

        # PWM frequency (Hz)
        self.pwm_frequency = pwm_frequency

        # Reverse motor direction
        self.invert = invert

        if GPIO.getmode() is None:
            GPIO.setmode(GPIO.BOARD)
        else:
            pass

        for pin in self.pins:  # Set motor pins as outputs
            GPIO.setup(pin, GPIO.OUT)

        self.motor = GPIO.PWM(  # set first pin as PWM and set freq
            self.pins[1], self.pwm_frequency
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
        """_summary_

        """
        GPIO.output(self.pins[0], False)
        self.motor.stop()
        GPIO.cleanup(self.pins)
