import time
import socket
import wheels

port = 9999

socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
socket.bind(("", port))

wheel = wheels.Wheel(1, 0x43) 	                                # Right Motor (ch2)
# wheel = wheels.Wheel(2, 0x40) 	                                # Left Motor (ch1)

start_time = time.time()

packet = ''

while 1:
    # request, ip = socket.recvfrom(1024)
    # packet = str(round(time.time() - start_time,3))+","+str(wheel.getSpeed())
    print(wheel.getSpeed())
    # socket.sendto(packet.encode(), ip)
