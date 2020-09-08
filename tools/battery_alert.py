# This code retrieves information from the onboard analog-to-digital converter
# on the beaglebone blue.
# Uses rcpy library.  Documentation at guitar.ucsd.edu/rcpy/rcpy.pdf

# Import External Libraries
import time                 # for handing timing
import numpy as np          # for handling arrays
import rcpy                 # for driving peripherals on beaglebone blue
from rcpy._adc import *     # import functions in rcpy adc

import glob

class ADC:

    def __init__(self):
        pass

    def read(self, channel):
        self.channel_voltage = get_voltage(channel)
        return self.channel_voltage

    def dcJack(self):
        self.jack_voltage = get_dc_jack_voltage()
        return self.jack_voltage


if __name__ == "__main__":

    adc = ADC()

    while True:
        shells = glob.glob("/dev/pts/*")

        if '/dev/pts/ptmx' in shells:
            shells.remove('/dev/pts/ptmx')

        if adc.dcJack() < 12:
            for shell in shells:
                with open(shell, "w") as term:
                    term.write("SCUTTLE Warning - Low Battery. ({} v)\n\r".format(round(adc.dcJack(), 2)))

        time.sleep(60)

