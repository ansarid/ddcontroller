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

            self.pins = pins                                # First pin will be digital and second pin will be PWM
            self.duty = 0                                   # Initial Duty %
            self.freqency = freqency                        # PWM freqency (Hz)
            self.invert = invert                            # Reverse motor direction? Duty of 1 becomes -1 and duty of -1 becomes 1

            for pin in pins:                                # Set motor pins as outputs
                GPIO.setup(pin, GPIO.OUT)

            self.motor = GPIO.PWM(pins[1], self.freqency)   # set first pin as PWM and set freq
            self.motor.start(self.duty)

        def setDuty(self, duty):
            self.duty = round(sorted((-1, float(duty), 1))[1], 2)  # Make sure duty is between -1 and 1

            duty = self.duty*100

            GPIO.output(self.pins[0], duty < 0)            # Set direction pin high if duty cycle is negative

            if duty == 0:
                GPIO.output(self.pins[0], False)
                self.motor.ChangeDutyCycle(0)

            else:
                self.motor.ChangeDutyCycle(duty if (duty > 0) else abs(100+duty))

        def stop(self):
            GPIO.output(self.pins[0], False)
            self.motor.stop()
            GPIO.cleanup(self.pins)
