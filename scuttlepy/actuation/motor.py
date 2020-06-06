#!/usr/bin/python3

# This example drives the right and left motors.
# Intended for Beaglebone Blue hardware.
# This example uses rcpy library. Documentation: guitar.ucsd.edu/rcpy/rcpy.pdf

# Import external libraries
import rcpy
import rcpy.motor as motor
import time                                     # only necessary if running this program as a loop
import numpy as np                              # for clip function

# NOTE: THERE ARE 4 OUTPUTS.  3 & 4 ACCESSIBLE THROUGH diode & accy functions


class Motor:

    def __init__(self, channel, invert=False):

        self.channel = channel
        self.duty = 0
        self.invert = invert

        rcpy.set_state(rcpy.RUNNING)
        motor.set(self.channel, self.duty)

    def setDuty(self, duty):
        if rcpy.get_state() == rcpy.RUNNING:        # execute loop when rcpy is ready
            if not self.invert:
                self.duty = duty
            else:
                self.duty = -1 * duty

            motor.set(self.channel, self.duty)

    def diode(self, state, channel):                # takes argument in range [0,1]
        np.clip(self.state, 0, 1)                   # limit the output, disallow negative voltages
        if rcpy.get_state() == rcpy.RUNNING:        # execute loop when rcpy is ready
            motor.set(self.channel, self.state)

    def accy(self, state, channel):                 # takes argument in range [-1,1]
        if rcpy.get_state() == rcpy.RUNNING:        # execute loop when rcpy is ready
            motor.set(self.channel, self.state)


if __name__ == "__main__":

    l_motor = Motor(1) 	                                # Left Motor (ch1)
    r_motor = Motor(2) 	                                # Right Motor (ch2)

    while rcpy.get_state() != rcpy.EXITING:     # exit loop if rcpy not ready
        if rcpy.get_state() == rcpy.RUNNING:    # execute loop when rcpy is ready
            print("motors.py: driving fwd")
            l_motor.setDuty(0.7)
            r_motor.setDuty(0.7)
            time.sleep(15)
            print("motors.py: driving reverse")
            l_motor.setDuty(-0.7)
            r_motor.setDuty(-0.7)
            time.sleep(15)
