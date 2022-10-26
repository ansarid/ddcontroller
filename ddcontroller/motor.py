#!/usr/bin/env python3

'''
This file is part of the ddcontroller library (https://github.com/ansarid/ddcontroller).
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

    def __init__(self, pins, pwm_frequency, initial_duty=0, decay_mode='FAST', invert=False, rpm=175):
        """_summary_

        Args:
            digital_pin (int, optional): _description_.
            pwm_pin (int, optional): _description_.
            pwm_frequency (int, optional): _description_.
            invert (bool, optional): _description_. Defaults to False.
        """

        # self.digital_pin = digital_pin
        # self.pwm_pin = pwm_pin
        self.pins = pins
        self._pins = []

        # First pin will be digital and second pin will be PWM
        # self.pins = (self.digital_pin, self.pwm_pin)

        # Initial Duty %
        self.duty = initial_duty

        # PWM frequency (Hz)
        self.pwm_frequency = pwm_frequency

        # Decay Mode (FAST/SLOW)
        self.decay_mode = decay_mode

        # Reverse motor direction
        self.invert = invert

        # Motor RPM
        self.rpm = rpm

        # Min motor Duty
        self.min_duty = 0

        # Max motor Duty
        self.max_duty = 1

        if GPIO.getmode() is None:
            GPIO.setmode(GPIO.BOARD)
        else:
            pass

        for pin in self.pins:  # Set motor pins as outputs
            GPIO.setup(pin, GPIO.OUT)
            self._pins.append(GPIO.PWM(pin, self.pwm_frequency))

        for pin in self._pins:
            pin.start(self.duty)

    def set_pwm_frequency(self, frequency):
        self.pwm_frequency = frequency
        for pin in self._pins:
            pin.ChangeFrequency(self.pwm_frequency)

    def set_duty(self, duty):
        """_summary_

        Args:
            duty (_type_): _description_
        """
        self.duty = round(  # Make sure duty is between -1 and 1
            sorted((-1, float(duty), 1))[1], 2
        )

        duty = self.duty * 100

        if self.decay_mode is 'SLOW':

            if duty == 0:
                for pin in self._pins:
                    pin.ChangeDutyCycle(100)

            elif duty > 0:
                self._pins[0].ChangeDutyCycle(100)
                self._pins[1].ChangeDutyCycle(100-duty)

            elif duty < 0:
                self._pins[0].ChangeDutyCycle(100-abs(duty))
                self._pins[1].ChangeDutyCycle(100)

        elif self.decay_mode is 'FAST':

            if duty == 0:
                for pin in self._pins:
                    pin.ChangeDutyCycle(0)

            elif duty > 0:
                self._pins[0].ChangeDutyCycle(0)
                self._pins[1].ChangeDutyCycle(duty)

            elif duty < 0:
                self._pins[0].ChangeDutyCycle(abs(duty))
                self._pins[1].ChangeDutyCycle(0)

    def stop(self):
        """_summary_

        """

        for pin in self._pins:
            pin.stop()

        GPIO.cleanup(self.pins)
