#!/usr/bin/python3

# This program reads information from the onboard Temperature & Pressure
# sensors on the beaglebone blue. Before running the code, you need to
# install required library using the command:
# sudo pip3 install bmp280

import time
from smbus import SMBus         # library for accessing i2c devices through python
from bmp280 import BMP280       # library dedicated to BMP280 sensor


class BMP:
    def __init__(self, bus=2):
        self.bus = SMBus(bus)
        self.bmp280 = BMP280(i2c_dev=self.bus)

    # get the sensor temp (C)
    def temp(self):
        self.temperature = self.bmp280.get_temperature()
        return self.temperature

    # get the ambient pressure (kPa)
    def pressure(self):
        self.pres = self.bmp280.get_pressure()*0.1
        return self.pres

    # get the estimated altitude (m) (not calibrated)
    # altitude is not reliable unless you know how to calibrate it
    def altitude(self):
        self.alt = self.bmp280.get_altitude()
        return self.alt


if __name__ == "__main__":

    bmp = BMP()

    while True:
        t = bmp.temp()
        p = bmp.pressure()
        a = bmp.altitude()
        print(f"Temperature (C): {t}\t Pressure (kPa): {p}")
        time.sleep(0.5)
