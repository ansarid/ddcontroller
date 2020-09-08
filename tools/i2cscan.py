#!/usr/bin/env python

import subprocess

def scanI2C(bus):
    cmd = ["i2cdetect", "-y", "-r", str(bus)]

    i2c_output = subprocess.check_output(cmd).decode("utf-8").split('\n')

    del i2c_output[0]
    del i2c_output[-1]

    i2c_output = [line[4:].strip().replace(' ', '').split('--') for line in i2c_output]
    i2c_output = [["0x"+i for i in line if i is not ''] for line in i2c_output]
    i2c_output = [[int(device, 16) for device in line] for line in i2c_output if line != []][0]

    return i2c_output

for address in scanI2C(1):
    print(hex(address))