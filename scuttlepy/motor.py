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
                if not self.invert:
                    self.duty = duty
                else:
                    self.duty = -1 * duty                   # Invert duty cycle

                motor.set(self.channel, self.duty)

elif detector.board.RASPBERRY_PI_40_PIN:

    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)

    class Motor:

        def __init__(self, pins, invert=False, freqency=150):

            self.pins = pins                                # First pin will be PWM and second pin will be digital
            self.duty = 0                                   # Initial Duty
            self.freqency = freqency                        # PWM freqency (Hz)
            self.invert = invert                            # Reverse motor direction? Duty of 1 becomes -1 and duty of -1 becomes 1

            for pin in pins:
                GPIO.setup(pin, GPIO.OUT)

            self.motor = GPIO.PWM(pins[0], self.freqency)

        def setDuty(self, duty):

            self.duty = duty

            if not self.invert:
                GPIO.output(self.pins[1], self.duty < 0)
            else:
                GPIO.output(self.pins[1], self.duty > 0)

            self.motor.ChangeDutyCycle(abs(self.duty*100))

elif detector.board.JETSON_NANO:

    import Jetson.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)

    class Motor:

        def __init__(self, pins, invert=False, freqency=150):

            self.pins = pins                                # First pin will be PWM and second pin will be digital
            self.duty = 0                                   # Initial Duty
            self.freqency = freqency                        # PWM freqency (Hz)
            self.invert = invert                            # Reverse motor direction? Duty of 1 becomes -1 and duty of -1 becomes 1

            for pin in pins:
                GPIO.setup(pin, GPIO.OUT)

            self.motor = GPIO.PWM(pins[0], self.freqency)

        def setDuty(self, duty):

            self.duty = duty

            if not self.invert:
                GPIO.output(self.pins[1], self.duty < 0)
            else:
                GPIO.output(self.pins[1], self.duty > 0)

            self.motor.ChangeDutyCycle(abs(self.duty*100))


if __name__ == "__main__":

    import time

    if detector.board.BEAGLEBONE_BLUE:

        l_motor = Motor(1) 	                                # Create Left Motor Object (ch1)
        r_motor = Motor(2) 	                                # Create Right Motor Object (ch2)

    elif detector.board.RASPBERRY_PI_40_PIN or detector.board.JETSON_NANO:

        l_motor = Motor((15,16)) 	                                # Create Left Motor Object (pwm, digital)
        r_motor = Motor((11,12)) 	                                # Create Right Motor Object (pwm, digital)

    else:
        print('Unsupported Platform "', detector.board.id, '"!')
        exit()

    try:
        while True:

            print("motors.py: driving fwd")
            l_motor.setDuty(1)                              # Set left motor duty cycle to 1
            r_motor.setDuty(1)                              # Set right motor duty cycle to 1
            time.sleep(5)                                   # Wait 5 seconds

            print("motors.py: driving reverse")
            l_motor.setDuty(-1)                             # Set left motor duty cycle to -1
            r_motor.setDuty(-1)                             # Set right motor duty cycle to -1
            time.sleep(5)                                   # Wait 5 seconds

    except KeyboardInterrupt:

        l_motor.setDuty(0)
        r_motor.setDuty(0)
