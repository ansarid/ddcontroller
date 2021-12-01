import time
import math
import socket
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button, RadioButtons

plt.style.use('dark_background')
class network:

    ip = "192.168.1.14"
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

l_targets = [0]
r_targets = [0]

l_error = [0]
r_error = [0]

timestamps = [0]

speedTarget = round(9, 2)

startTime = time.time()

def animate(i):

    global speedTarget

    try:

        p = 0.015
        i = 0.025
        d = 0.0

        # p = 0.004
        # i = 0.025
        # d = 0

        message = (str(speedTarget)+","+str(p)+","+str(i)+","+str(d)).encode()
        socket.sendto(message, (network.ip, network.port))
        data, ip = socket.recvfrom(4000)
        data = data.decode('utf-8').split(',')

        # print(data[0])

        if float(data[0]) >= 3:
            message = str(0).encode()


        timestamps.append(float(data[0]))
        l_speeds.append(float(data[1]))
        r_speeds.append(float(data[2]))
        l_dutys.append(float(data[3])*10)
        r_dutys.append(float(data[4])*10)

        l_error.append(float(data[5]))
        r_error.append(float(data[6]))

        l_targets.append(float(data[7]))
        r_targets.append(float(data[8]))

        if len(l_speeds) > 1000:
            l_speeds.pop(0)
            r_speeds.pop(0)

            l_dutys.pop(0)
            r_dutys.pop(0)

            timestamps.pop(0)

        if len(l_speeds) > 10:
            l_speed_av.append(np.average(l_speeds[-10:]))
            r_speed_av.append(np.average(r_speeds[-10:]))
        else:
            l_speed_av.append(np.average(l_speeds))
            r_speed_av.append(np.average(r_speeds))

        # print(timestamps[-1])

        if timestamps[-1] < 5:
            speedTarget = speedTarget

        elif round(timestamps[-1]) == 5:
            speedTarget = 4

        elif round(timestamps[-1]) == 10:
            speedTarget = 7

        elif round(timestamps[-1]) == 15:
            speedTarget = 2

        elif round(timestamps[-1]) > 20:
            speedTarget = -5

        pid_plot.clear()

        textstr = '\n'.join((
            r'P = %.4f' % (p, ),
            r'I  = %.4f' % (i, ),
            r'D = %.4f' % (d, )))

        plt.grid(color="dimgrey")

        # these are matplotlib.patch.Patch properties
        props = dict(boxstyle='round', facecolor='black', alpha=0.1)

        # place a text box in upper left in axes coords
        # plt.text(l_speeds[-1], 0.95, textstr, fontsize=15,
        #         verticalalignment='top', bbox=props)

        plt.text(2, 1.5, textstr, fontsize=15,
                verticalalignment='top', bbox=props)

        # pid_plot.axhline(speed, color='blue', lw=2)


        pid_plot.plot(timestamps, l_speeds, color='red')   # Colored Points
        # pid_plot.scatter(timestamps, r_speeds, color='red')   # Colored Points
        # pid_plot.plot(timestamps, r_speeds, color='red')   # Colored Points

        pid_plot.plot(timestamps, l_targets, color='blue')   # Colored Points
        # pid_plot.plot(timestamps, r_targets, color='blue')   # Colored Points

        pid_plot.plot(timestamps, l_dutys, color='yellow')   # Colored Points
        # pid_plot.plot(timestamps, r_dutys, color='yellow')   # Colored Points

        # pid_plot.plot(timestamps, l_speed_av, color='yellow')   # Colored Points
        # pid_plot.plot(timestamps, r_speed_av)   # Colored Points

        # pid_plot.plot(timestamps, r_error, color='yellow')   # Colored Points
        # pid_plot.scatter(timestamps, r_error, color='yellow')   # Colored Points

        print(timestamps[-1], ",", speedTarget)

    except Exception as e:
        # print(e)
        pass

ani = animation.FuncAnimation(fig, animate, interval=1)

plt.show()
# plt.savefig(str("P")+str(p)+str("_I")+str(i)+str("_D")+str(d)+'.png')

message = str(0).encode()
socket.sendto(message, (network.ip, network.port))
message = 'q'.encode()
socket.sendto(message, (network.ip, network.port))
