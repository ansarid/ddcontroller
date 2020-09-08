import time
import numpy as np
import mpu
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

my_filter = KalmanFilter(dim_x=2, dim_z=1)

my_filter.x = np.array([[0.],
                        [0.]])       # initial state (location and velocity)

my_filter.F = np.array([[1.,1.],
                        [0.,1.]])    # state transition matrix

my_filter.H = np.array([[1.,0.]])    # Measurement function
my_filter.P *= 10.                 # covariance matrix
my_filter.R = 100                      # state uncertainty
# my_filter.Q = Q_discrete_white_noise(2, 0.1, .1) # process uncertainty

imu = mpu.IMU()

startTime = time.time()

print('time',',', 'rawHeading', ',', 'filteredHeading')

while True:

    rawHeading = imu.getHeading()

    my_filter.predict()
    my_filter.update(rawHeading)

    # do something with the output

    x = my_filter.x
    # print(x[0])
    print(time.time()-startTime,',', rawHeading, ',', x[0][0])
    time.sleep(0.01)
