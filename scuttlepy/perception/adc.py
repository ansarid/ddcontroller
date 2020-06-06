# This code retrieves information from the onboard analog-to-digital converter
# on the beaglebone blue.
# Uses rcpy library.  Documentation at guitar.ucsd.edu/rcpy/rcpy.pdf

# Import External Libraries
import time                 # for handing timing
import numpy as np          # for handling arrays
import rcpy                 # for driving peripherals on beaglebone blue
from rcpy._adc import *     # import functions in rcpy adc


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
    while True:
        adc = ADC()
        channel_voltage = adc.dcJack()
        print(channel_voltage)
        time.sleep(0.5)
