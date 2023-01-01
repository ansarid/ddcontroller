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

    def __init__(self, pins, pwm_frequency, initial_duty=0, decay_mode='FAST', invert=False, rpm=200):

        """Class for controlling a motor using the Raspberry Pi's GPIO pins.

        Args:
            pins (list): A list of the GPIO pins that will be used to control the motor.
            pwm_frequency (int): The frequency of the pulse width modulation (PWM) signal
                that will be used to control the motor.
            initial_duty (int, optional): The initial duty cycle of the PWM signal. Defaults to 0.
            decay_mode (str, optional): The decay mode of the motor. Can be either "FAST" or
                "SLOW". Defaults to "FAST".
            invert (bool, optional): A boolean value indicating whether the motor's direction
                should be reversed. Defaults to False.
            rpm (int, optional): The speed of the motor in rotations per minute. Defaults to 200.
        """

        self.pins = pins
        self._pins = []

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
        """Sets the frequency of the PWM signal.

        Args:
            frequency (int): The new frequency of the PWM signal.
        """
        self.pwm_frequency = frequency
        for pin in self._pins:
            pin.ChangeFrequency(self.pwm_frequency)

    def set_duty(self, duty):
        """Sets the duty cycle of the PWM signal.

        Args:
            duty (float): The new duty cycle of the PWM signal, from -1 to 1.
        """
        self.duty = round(  # Make sure duty is between -1 and 1
            sorted((-1, float(duty), 1))[1], 2
        )

        duty = self.duty * 100

        if self.decay_mode == 'SLOW':

            if duty == 0:
                for pin in self._pins:
                    pin.ChangeDutyCycle(100)

            elif duty > 0:
                self._pins[0].ChangeDutyCycle(100)
                self._pins[1].ChangeDutyCycle(100-duty)

            elif duty < 0:
                self._pins[0].ChangeDutyCycle(100-abs(duty))
                self._pins[1].ChangeDutyCycle(100)

        elif self.decay_mode == 'FAST':

            if duty == 0:
                for pin in self._pins:
                    pin.ChangeDutyCycle(0)

            elif duty > 0:
                self._pins[0].ChangeDutyCycle(0)
                self._pins[1].ChangeDutyCycle(duty)

            elif duty < 0:
                self._pins[0].ChangeDutyCycle(abs(duty))

        else:
            print('Invalid Decay Mode!')

    def stop(self):
        """Stops the motor by setting the duty cycle of the PWM signal to 0."""

        for pin in self._pins:
            pin.stop()

        GPIO.cleanup(self.pins)