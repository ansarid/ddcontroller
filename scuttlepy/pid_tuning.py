import time
import socket
import wheels

port = 9999

socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
socket.bind(("", port))

# l_wheel = wheels.Wheel(1, 0x43, KP=0.008, KI=0.5, KD=0.0005, invert_encoder=True) 	                # Right Motor (ch2)
l_wheel = wheels.Wheel(1, 0x43, KP=0.004, KI=0.12, KD=0.0005, invert_encoder=True) 	                # Right Motor (ch2)
r_wheel = wheels.Wheel(2, 0x40, KP=0.004, KI=0.12, KD=0.0005) 	                                    # Left Motor (ch1)

# max p contribution is 0.004*6 = 0.024
# max i contribution is 0.12*6*(dt) where dt is 0.080 = 0.058 * (number of samples)
# power required to achieve 0.8 duty cycle = 13 samples * 0.058 = 0.75

start_time = time.time()

packet = ''

while 1:
    request, ip = socket.recvfrom(1024)
    if request.decode('utf-8') == 'q':
        exit()

    l_wheel.setAngularVelocity(float(request))
    # r_wheel.setAngularVelocity(float(request))

    packet = str(round(time.time() - start_time,3))+","+str(l_wheel.getAngularVelocity())+","+str(r_wheel.getAngularVelocity())
    socket.sendto(packet.encode(), ip)
