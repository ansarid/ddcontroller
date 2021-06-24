import csv
import time
import wheels
import numpy as np

l_wheel = wheels.Wheel((32,29), 0x40, openLoop=True, invert_encoder=True)                     # Left Motor  (ch1)
r_wheel = wheels.Wheel((33,31), 0x41, openLoop=True) 	                                        # Right Motor (ch2)

# FIND AVERAGE SPEED WITH MAX DUTY
averageMax = []
averageTime = 15    # How much time to spend gathering data for average
waitTime = 10       # Time to wait before taking data
startTime = time.monotonic()

l_wheel.setAngularVelocity(10)
r_wheel.setAngularVelocity(10)

while (time.monotonic() - startTime) < waitTime:
    pass

while (time.monotonic()-startTime) < averageTime:
    # print(l_wheel.getAngularVelocity())
    averageMax.append([l_wheel.getAngularVelocity(), r_wheel.getAngularVelocity()])

average = np.array(averageMax)
average = np.average(average, axis=0)
print('\nAverage:',average)

# FIND MOTOR SLOPES
# data = np.array()
stepTime = 5    # Time to spend gathering data at each duty cycle

with open('data.csv', 'w', newline='') as csvfile:
    fieldnames = ['Target Velocity (rad/s)', 'L Velocity (rad/s)', 'R Velocity (rad/s)']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

    l_wheel.setAngularVelocity(average[0])
    r_wheel.setAngularVelocity(average[1])

    while l_wheel.getAngularVelocity() < average[0] or r_wheel.getAngularVelocity() < average[1]:
        pass

    for velocity in np.arange(15, -15, -0.5):

        velocity = round(velocity,2)

        print(velocity)
        startTime = time.monotonic()

        l_wheel.setAngularVelocity(velocity)
        r_wheel.setAngularVelocity(velocity)

        average = np.array([l_wheel.getAngularVelocity(), r_wheel.getAngularVelocity()])

        while (time.monotonic()-startTime) < stepTime:
            # print(l_wheel.getAngularVelocity())
            average = np.vstack((average,np.array([l_wheel.getAngularVelocity(), r_wheel.getAngularVelocity()])))

        average = np.round(np.average(average, axis=0),2)
        writer.writerow({'Target Velocity (rad/s)':velocity, 'L Velocity (rad/s)':average[0], 'R Velocity (rad/s)':average[1]})

l_wheel.setAngularVelocity(0)
r_wheel.setAngularVelocity(0)
