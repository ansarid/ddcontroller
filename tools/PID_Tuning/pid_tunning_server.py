import threading
import time
import socket
from scuttlepy import wheels

port = 9999

socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
socket.bind(("", port))


packet = ''

request = [0, 0, 0, 0]

request, ip = socket.recvfrom(1024)
start_time = time.monotonic()

l_wheel = wheels.Wheel(1, 0x40, KP=0, KI=0, KD=0, invert_encoder=True)
r_wheel = wheels.Wheel(2, 0x41, KP=0, KI=0, KD=0)



class WheelAssembly:

    def __init__(self):

        self.l_speed = 0
        self.r_speed = 0

        self.stopped = False

        self.t = threading.Thread(target=self.wheelsThread)
        self.t.start()

    def wheelsThread(self):

        while not self.stopped:
            startTime=time.monotonic()
            l_wheel.setAngularVelocity(self.l_speed)
            r_wheel.setAngularVelocity(self.r_speed)
            print(round(1/((time.monotonic()-startTime)/2),3))

    def stop(self):
        self.stopped = True
        l_wheel.setAngularVelocity(0)
        r_wheel.setAngularVelocity(0)


wheelAssembly = WheelAssembly()

start_time = time.monotonic()

try:
    while True:
        otherTime = time.monotonic()
        request, ip = socket.recvfrom(1024)
        # print(request)
        if request.decode('utf-8') == 'q':
            exit()

            # print(request[0])

        else:

            request = request.decode('utf-8').split(',')
            # l_wheel.setAngularVelocity(-1*float(request[0]))

            # if (time.monotonic() - start_time) >= 40:
            wheelAssembly.l_speed = int(request[0])
            wheelAssembly.r_speed = int(request[0])

            # else:
            #     l_wheel.setAngularVelocity(float(request[0]))
            #     # l_wheel.setAngularVelocity(0)
            #     r_wheel.setAngularVelocity(float(request[0]))
            #     # l_wheel.motor.setDuty(0.6)
            # time.sleep(0.060)

            r_wheel.pid.setKp(float(request[1]))
            r_wheel.pid.setKi(float(request[2]))
            r_wheel.pid.setKd(float(request[3]))

            l_wheel.pid.setKp(float(request[1]))
            l_wheel.pid.setKi(float(request[2]))
            l_wheel.pid.setKd(float(request[3]))

            timestamp = round(time.monotonic() - start_time, 3)
            # print(timestamp)

        packet = str(timestamp)+\
            ","+str(l_wheel.speed)+\
            ","+str(r_wheel.speed)+\
            ","+str(l_wheel.motor.duty)+\
            ","+str(r_wheel.motor.duty)+\
            ","+str(l_wheel.pid.error)+\
            ","+str(r_wheel.pid.error)+\
            ","+str(l_wheel.pid.SetPoint)+\
            ","+str(r_wheel.pid.SetPoint)

        # print(timestamp, ",", r_wheel.pid.SetPoint)

        socket.sendto(packet.encode(), ip)

        # print(packet)

        if (time.monotonic() - start_time) >= 40:
            break

        # print(round(time.monotonic()-otherTime,3))

except KeyboardInterrupt:
    wheelAssembly.l_speed = 0
    wheelAssembly.r_speed = 0
    wheelAssembly.stop()

except:
    wheelAssembly.l_speed = 0
    wheelAssembly.r_speed = 0
    wheelAssembly.stop()
