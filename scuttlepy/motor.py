#!/usr/bin/python3

# This example drives the right and left motors.
# Intended for Beaglebone Blue hardware.
# This example uses rcpy library. Documentation: guitar.ucsd.edu/rcpy/rcpy.pdf

# from adafruit_platformdetect import Detector
# detector = Detector()
# if detector.board.BEAGLEBONE_BLUE:

# Import external libraries
import rcpy
import rcpy.motor as motor


class Motor:

    def __init__(self, channel, invert=False):

        self.channel = channel
        self.duty = 0
        self.invert = invert

        rcpy.set_state(rcpy.RUNNING)
        motor.set(self.channel, self.duty)

    def setDuty(self, duty):
        if rcpy.get_state() == rcpy.RUNNING:            # execute loop if rcpy is running
            if not self.invert:
                self.duty = duty
            else:
                self.duty = -1 * duty

            motor.set(self.channel, self.duty)


if __name__ == "__main__":

    import time

    l_motor = Motor(1) 	                                # Left Motor (ch1)
    r_motor = Motor(2) 	                                # Right Motor (ch2)

    while rcpy.get_state() != rcpy.EXITING:             # exit loop if rcpy not ready
        if rcpy.get_state() == rcpy.RUNNING:            # execute loop when rcpy is ready
            print("motors.py: driving fwd")
            l_motor.setDuty(0.7)
            r_motor.setDuty(0.7)
            time.sleep(15)
            print("motors.py: driving reverse")
            l_motor.setDuty(-0.7)
            r_motor.setDuty(-0.7)
            time.sleep(15)
