import threading
import time
import socket
from scuttlepy import wheels

port = 9999

socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
socket.bind(("", port))

l_wheel = wheels.Wheel(1, 0x41, KP=0, KI=0, KD=0)
r_wheel = wheels.Wheel(2, 0x40, KP=0, KI=0, KD=0, invert_encoder=True)

packet = ''

request = [0, 0, 0, 0]

request, ip = socket.recvfrom(1024)
start_time = time.monotonic()


while 1:
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
        l_wheel.setAngularVelocity(float(request[0]))
        r_wheel.setAngularVelocity(-1*float(request[0]))

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

    print(round(time.monotonic()-otherTime,3))
