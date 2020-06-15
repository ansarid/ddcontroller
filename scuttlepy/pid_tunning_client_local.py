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

l_targets = [0]
r_targets = [0]

l_error = [0]
r_error = [0]

timestamps = [0]

speedTarget = round(4.0, 2) #6.28 rad/sec

def animate(i):

    global speedTarget

    try:

        p = 0.07
        i = 2
        d = 0

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
        l_dutys.append(float(data[3])*4)
        r_dutys.append(float(data[4])*4)

        l_error.append(float(data[5]))
        r_error.append(float(data[6]))

        l_targets.append(float(speedTarget))
        r_targets.append(float(speedTarget))

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

        if timestamps[-1] >= 4:
            speedTarget = 0

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

        # pid_plot.axhline(speed, color='blue', lw=2)


        # pid_plot.plot(timestamps, l_speeds, color='red')   # Colored Points
        pid_plot.scatter(timestamps, r_speeds, color='red')   # Colored Points
        pid_plot.plot(   timestamps, r_speeds, color='red')   # Colored Points

        pid_plot.plot(timestamps, r_targets, color='blue')   # Colored Points
        # pid_plot.plot(timestamps, l_targets, color='blue')   # Colored Points

        pid_plot.plot(timestamps, r_dutys, color='green')   # Colored Points
        # pid_plot.plot(timestamps, l_dutys, color='green')   # Colored Points

        # pid_plot.plot(timestamps, l_speed_av, color='yellow')   # Colored Points
        # pid_plot.plot(timestamps, r_speed_av)   # Colored Points

        pid_plot.plot(timestamps, r_error, color='yellow')   # Colored Points
        pid_plot.scatter(timestamps, r_error, color='yellow')   # Colored Points

    except Exception as e:
        # print(e)
        pass

ani = animation.FuncAnimation(fig, animate, interval=50)

plt.show()
# plt.savefig(str("P")+str(p)+str("_I")+str(i)+str("_D")+str(d)+'.png')

message = str(0).encode()
socket.sendto(message, (network.ip, network.port))
message = 'q'.encode()
socket.sendto(message, (network.ip, network.port))