#!/usr/bin/python3

# This program accesses info from the Blue's onboard sensor, MPU9250
# It reads temp, accelerometer, gyro, and magnetometer data from the sensor.
# Uses RCPY library.  See guitar.ucsd.edu/rcpy/rcpy.pdf for documentation

# Import external librarires
import time                                     # for time.sleep function
import numpy as np                              # for working with matrices
import rcpy                                     # import rcpy library (this automatically initializes robotics cape)
import rcpy.mpu9250 as mpu9250                  # for mpu sensor functions


class MPU:

    def __init__(self):

        rcpy.set_state(rcpy.RUNNING)                    # set state to rcpy.RUNNING

        mpu9250.initialize(enable_dmp=True,
                           dmp_sample_rate=100,
                           enable_fusion=True,
                           enable_magnetometer=True)

    def readAccel(self):
        axes = mpu9250.read_accel_data()            # returns x, y, z acceleration (m/s^2)
        # axes = np.round(axes, 3)                  # round values to 3 decimals
        return(axes)

    def readAll(self):
        data = mpu9250.read()                       # this command returns a string with many parameters.
        return(data)

    def readTemp(self):
        temp = mpu9250.read_imu_temp()              # returns just temperature (deg C)
        return(temp)

    def readMag(self):
        mag = mpu9250.read_mag_data()               # gets x,y,z mag values (microtesla)
        # mag = np.round(mag, 1)                    # round values to 1 decimal
        return(mag)

    def readGyro(self):
        gyro = mpu9250.read_gyro_data()             # returns 3 axes gyro data (deg/s)
        # gyro = np.round(gyro, 2)
        return(gyro)


# if __name__ == "__main__":
    
#     mpu = MPU()
#     while True:
#         if rcpy.get_state() == rcpy.RUNNING:    # verify the rcpy package is running
#             myTemp = mpu.readTemp()
#             myMag = mpu.readMag()
#             myGyro = mpu.readGyro()
#             myAccel = mpu.readAccel()
#             print("temperature (C):", myTemp,"\t mag (μT):", myMag,"\t gyro(deg/s)",myGyro, "\t accel(m/s^2):", myAccel)
#         time.sleep(0.5)
