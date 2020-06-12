import time
import socket
import wheels

port = 9999

socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
socket.bind(("", port))

l_wheel = wheels.Wheel(1, 0x40, KP=0, KI=0, KD=0, invert_encoder=True) 	                # Right Motor (ch2)
r_wheel = wheels.Wheel(2, 0x41, KP=0, KI=0, KD=0) 	                                    # Left Motor (ch1)

start_time = time.time()

packet = ''

while 1:
    request, ip = socket.recvfrom(1024)
    if request.decode('utf-8') == 'q':
        exit()

    else:

        request = request.decode('utf-8').split(',')

        r_wheel.pid.setKp(float(request[1]))
        r_wheel.pid.setKi(float(request[2]))
        r_wheel.pid.setKd(float(request[3]))

        l_wheel.pid.setKp(float(request[1]))
        l_wheel.pid.setKi(float(request[2]))
        l_wheel.pid.setKd(float(request[3]))

        l_wheel.setAngularVelocity(float(request[0]))
        r_wheel.setAngularVelocity(float(request[0]))
        # l_wheel.motor.setDuty(0.6)


    packet = str(round(time.time() - start_time,3))+","+str(l_wheel.getAngularVelocity())+","+str(r_wheel.getAngularVelocity())+","+str(l_wheel.motor.duty)
    socket.sendto(packet.encode(), ip)
