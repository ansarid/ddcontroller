import math
import time
import RTIMU

class IMU:

    def __init__(self, settingsFile='RTIMULib'):

        self.heading = 0
        self.pose = [0,0,0]

        self.settings = RTIMU.Settings(settingsFile)
        self.imu = RTIMU.RTIMU(self.settings)

        if (not self.imu.IMUInit()):
            return 1

        else:
            pass

        self.imu.setSlerpPower(0.02)
        self.imu.setGyroEnable(True)
        self.imu.setAccelEnable(True)
        self.imu.setCompassEnable(True)
        self.poll_interval = self.imu.IMUGetPollInterval()

        self.data = None

        while self.data is None:
            self.readIMU()

    def readIMU(self):
        if self.imu.IMURead():
            self.data = self.imu.getIMUData()
        else:
            pass
        return self.data

    def getPose(self):

        self.readIMU()

        pose = self.imu.getFusionData()

        self.pose = (math.degrees(pose[0]),
                     math.degrees(pose[1]),
                     math.degrees(pose[2])
                    )

        return self.pose

    def getHeading(self):

        heading = self.getPose()[2]

        self.heading = heading

        return self.heading


if __name__ == "__main__":

    imu = IMU()

    while True:

        print(imu.getHeading())
        print(imu.data)
        time.sleep(0.1)
