import time
import math
import socket
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation

plt.style.use('dark_background')
class network:

    ip = "192.168.1.83"
    port = 9999

try:

    socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    socket.settimeout(0.001)

except socket.error:

    print("Oops, something went wrong connecting the server!")
    exit()

fig = plt.figure()
pid_plot = plt.subplot()

l_speeds = [0]
r_speeds = [0]

l_speed_av = [0]
r_speed_av = [0]

l_dutys = [0]
r_dutys = [0]

timestamps = [0]


def animate(i):

    try:

        p = 0.07
        i = 1.5
        d = 0

        speed = round(math.pi*2, 2) #6.28 rad/sec
        message = (str(speed)+","+str(p)+","+str(i)+","+str(d)).encode()
        socket.sendto(message, (network.ip, network.port))
        data, ip = socket.recvfrom(4000)
        data = data.decode('utf-8').split(',')

        print(data[0])

        if float(data[0]) >= 3:
            message = str(0).encode()


        timestamps.append(float(data[0]))
        l_speeds.append(float(data[1]))
        r_speeds.append(float(data[2]))
        l_dutys.append(float(data[3])*speed)

        if len(l_speeds) > 500:
            l_speeds.pop(0)
            r_speeds.pop(0)

            l_dutys.pop(0)
            # r_dutys.pop(0)

            timestamps.pop(0)

        if len(l_speeds) > 10:
            l_speed_av.append(np.average(l_speeds[-10:]))
            r_speed_av.append(np.average(r_speeds[-10:]))
        else:
            l_speed_av.append(np.average(l_speeds))
            r_speed_av.append(np.average(r_speeds))

        pid_plot.clear()


        mu = 1
        median = 2
        sigma = 3

        textstr = '\n'.join((
            r'P = %.2f' % (p, ),
            r'I = %.2f' % (i, ),
            r'D = %.2f' % (d, )))

        plt.grid(color="dimgrey")

        # these are matplotlib.patch.Patch properties
        props = dict(boxstyle='round', facecolor='black', alpha=0.1)

        # place a text box in upper left in axes coords
        # plt.text(l_speeds[-1], 0.95, textstr, fontsize=15,
        #         verticalalignment='top', bbox=props)

        plt.text(2, 1.5, textstr, fontsize=15,
                verticalalignment='top', bbox=props)


        pid_plot.axhline(speed, color='red', lw=2)


        pid_plot.plot(timestamps, l_speeds, color='red')   # Colored Points
        #pid_plot.scatter(timestamps, l_speeds, color='red')   # Colored Points
        # pid_plot.plot(timestamps, r_speeds)   # Colored Points

        pid_plot.plot(timestamps, l_dutys, color='green')   # Colored Points

        pid_plot.plot(timestamps, l_speed_av, color='yellow')   # Colored Points
        # pid_plot.plot(timestamps, r_speed_av)   # Colored Points

    except Exception as e:
        print(e)

ani = animation.FuncAnimation(fig, animate, interval=50)

plt.show()
# plt.savefig(str("P")+str(p)+str("_I")+str(i)+str("_D")+str(d)+'.png')

message = str(0).encode()
socket.sendto(message, (network.ip, network.port))
message = 'q'.encode()
socket.sendto(message, (network.ip, network.port))