#This example reads left and right encoders and outputs the position (deg) to the terminal
# Left has address 40 and right has 41
# Code for Beagle Hardware

# Import internal libraries
import L2_log as log

# Import external libraries
import Adafruit_GPIO.I2C as Adafruit_I2C # for i2c communication functions
import time
import numpy as np # for handling arrays

encL = Adafruit_I2C.Device(0x40,1) # encoder i2c address
encR  = Adafruit_I2C.Device(0x41,1) # encoder i2c address

# the readEncs function communicates to one device in one function call
def readEnc(channel):
    try:
        # The AS5048B encoder gives a 14 bit angular reading
        if channel == 'L':
            msB = encL.readU16(0xFE) # capture the 8 msb's from encoder
            lsB = encL.readU16(0xFF) # capture the 6 lsb's from encoder
        elif channel == "R":
            msB = encR.readU16(0xFE) # capture the 8 msb's from encoder
            lsB = encR.readU16(0xFF) # capture the 6 lsb's from encoder
        # lsB can contribute  at most 1.4 degrees to the reading
        # for msB, perform bitwise operation to get true scaling of these bits
        msB = ((msB << 8) | (msB >> 8)) & 0xFFFF
        msB = ((msB & 0xFF00) >> 2) | ( msB & 0x3F)
        angle_raw = msB + lsB  # multiplying by 0.0219 will give degrees
    except:
        print('Warning (I2C): Could not read encoder ' + channel)
        angle_raw = 0  # set to zero, avoid sending wrong value
    return angle_raw   # the returned value must be scaled by ( 359deg / 2^14 )
    
# the READ function returns both encoder values (RAW).  Call this fcn from other programs.
def read():
    encLeft = readEnc('L') # call for left enc value
    encRight = readEnc('R') # call for right enc value
    encoders = np.array([encLeft,encRight]) # form array from left and right
    return encoders

# UNCOMMENT THIS SECTION TO USE ENCODER_EX2.PY AS A STANDALONE PROGRAM
# ------------------------------------------------------------------------------
# while 1:
#       encoders = read()
#       encoders = np.round( (encoders * (359 / 2**14) ), 2) # scale values to get degrees
#       print("encoders: ", encoders) # print the values
#       # SECTION FOR LOGGING --------------------------
#       log.uniqueFile(encoders[0], "encL.txt")
#       log.uniqueFile(encoders[1], "encR.txt")
#       time.sleep(0.10) # short delay
