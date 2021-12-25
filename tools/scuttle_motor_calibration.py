#!/usr/bin/python3

import time
import numpy as np
from collections import deque
from numpy.lib.function_base import average
from scuttlepy.constants import *
from scuttlepy.wheels import Wheel

settings = Settings()

leftWheel  = Wheel(settings.LEFT_WHEEL_MOTOR_PINS,                             # Create left wheel object
                   settings.I2C_BUS,
                   settings.LEFT_WHEEL_ENCODER_ADDRESS,
                   invertEncoder=settings.LEFT_WHEEL_ENCODER_INVERT,
                   invertMotor=settings.LEFT_WHEEL_MOTOR_INVERT,
                   )

rightWheel = Wheel(settings.RIGHT_WHEEL_MOTOR_PINS,                            # Create right wheel object
                   settings.I2C_BUS,
                   settings.RIGHT_WHEEL_ENCODER_ADDRESS,
                   invertEncoder=settings.RIGHT_WHEEL_ENCODER_INVERT,
                   invertMotor=settings.RIGHT_WHEEL_MOTOR_INVERT,
                   )

try:

    wheels = {'Left Wheel':leftWheel, 'Right Wheel':rightWheel}

    wheel_frequency = 50            # Needs to be fetched from config file
    minimum_velocity_change = 1     # Needs to be fetched from config file
    average_samples = 150           # Needs to be fetched from config file

    for wheel in wheels.keys():
        # print('\nCalibrating '+wheel+'...\nThis could take up to 2 minutes.\n')
        print('\n\n'+wheel+' Calibration:\n')
        # Test for minimum PWM required for forward motion on wheel
        for duty in range(100):

            duty /= 100
            velocities = deque([0], maxlen=average_samples)

            while np.average(velocities) < minimum_velocity_change and len(velocities) < average_samples:

                startTime = time.monotonic_ns()                                 # Record loop start time

                wheels[wheel].update()
                wheels[wheel].motor.setDuty(duty)
                velocities.append(wheels[wheel].getAngularVelocity())

                time.sleep(sorted([(1/wheel_frequency)-((time.monotonic_ns()-startTime)/1e9), 0])[1])    # Measure time since start and subtract from sleep time
                # print((time.monotonic_ns()-startTime)/1e6)                                      # Print loop time in ms

            if round(np.average(velocities), 2) > minimum_velocity_change:

                minimum_duty = duty
                minimum_angular_velocity = round(np.average(velocities), 2)

                # print(wheel,'Minimum Forward Duty:', minimum_duty,
                #       '\t',wheel,'Minimum Forward Angular Velocity:', minimum_angular_velocity)

                setattr(wheels[wheel], 'minimum_forward_duty', minimum_duty)
                setattr(wheels[wheel], 'maximum_forward_duty', 1)
                setattr(wheels[wheel], 'minimum_forward_angular_velocity', minimum_angular_velocity)

                print('minimum_forward_duty:', wheels[wheel].minimum_forward_duty)
                print('maximum_forward_duty:', wheels[wheel].maximum_forward_duty)             # I know this is silly
                print('minimum_forward_angular_velocity:', wheels[wheel].minimum_forward_angular_velocity)

                break

        velocities.clear()
        # Measure maximum forward wheel velocity
        while len(velocities) < average_samples:

            startTime = time.monotonic_ns()                                 # Record loop start time

            wheels[wheel].update()
            wheels[wheel].motor.setDuty(1)
            velocities.append(wheels[wheel].getAngularVelocity())

            time.sleep(sorted([(1/wheel_frequency)-((time.monotonic_ns()-startTime)/1e9), 0])[1])    # Measure time since start and subtract from sleep time
            # print((time.monotonic_ns()-startTime)/1e6)                                      # Print loop time in ms

        maximum_angular_velocity = round(np.average(velocities), 2)
        setattr(wheels[wheel], 'maximum_forward_angular_velocity', maximum_angular_velocity)
        print('maximum_forward_angular_velocity:', wheels[wheel].maximum_forward_angular_velocity)

        # results.append('maximum_forward_angular_velocity: '+str(maximum_angular_velocity))
        # print(wheel,'Maximum Forward Angular Velocity:', maximum_angular_velocity)

        velocities.clear()
        # Test for minimum PWM required for backward motion on wheel
        for duty in range(100):

            duty /= 100
            velocities = deque([0], maxlen=average_samples)

            while np.average(velocities) > -minimum_velocity_change and len(velocities) < average_samples:

                startTime = time.monotonic_ns()                                 # Record loop start time

                wheels[wheel].update()
                wheels[wheel].motor.setDuty(-duty)
                velocities.append(wheels[wheel].getAngularVelocity())

                time.sleep(sorted([(1/wheel_frequency)-((time.monotonic_ns()-startTime)/1e9), 0])[1])    # Measure time since start and subtract from sleep time
                # print((time.monotonic_ns()-startTime)/1e6)                                      # Print loop time in ms

            if round(np.average(velocities), 2) < -minimum_velocity_change:

                minimum_duty = duty
                minimum_angular_velocity = round(np.average(velocities), 2)

                # print(wheel,'Minimum Backward Duty:', minimum_duty,
                #       '\t',wheel,'Minimum Backward Angular Velocity:', minimum_angular_velocity)

                setattr(wheels[wheel], 'minimum_backward_duty', -minimum_duty)
                setattr(wheels[wheel], 'maximum_backward_duty', -1)             # I know this is silly
                setattr(wheels[wheel], 'minimum_backward_angular_velocity', minimum_angular_velocity)

                print('minimum_backward_duty:', wheels[wheel].minimum_backward_duty)
                print('maximum_backward_duty:', wheels[wheel].maximum_backward_duty)             # I know this is silly
                print('minimum_backward_angular_velocity:', wheels[wheel].minimum_backward_angular_velocity)

                velocities.clear()
                break

        velocities.clear()
        # Measure maximum backward wheel velocity
        while len(velocities) < average_samples:

            startTime = time.monotonic_ns()                                 # Record loop start time

            wheels[wheel].update()
            wheels[wheel].motor.setDuty(-1)
            velocities.append(wheels[wheel].getAngularVelocity())

            time.sleep(sorted([(1/wheel_frequency)-((time.monotonic_ns()-startTime)/1e9), 0])[1])    # Measure time since start and subtract from sleep time
            # print((time.monotonic_ns()-startTime)/1e6)                                      # Print loop time in ms

        maximum_angular_velocity = round(np.average(velocities), 2)

        setattr(wheels[wheel], 'maximum_backward_angular_velocity', maximum_angular_velocity)
        print('maximum_backward_angular_velocity:', wheels[wheel].maximum_backward_angular_velocity)

        # results.append('maximum_backward_angular_velocity: '+str(-maximum_angular_velocity))
        # print(wheel,'Maximum Backward Angular Velocity:', maximum_angular_velocity)
        velocities.clear()

        wheels[wheel].motor.setDuty(0)

        # Calculate max forward duty cycle for faster wheel that clips output to max of slower wheel
        # Calculate max backward duty cycle for faster wheel that clips output to max of slower wheel

        # Calculate min forward duty cycle for faster wheel that clips output to max of slower wheel
        # Calculate min backward duty cycle for faster wheel that clips output to max of slower wheel

        # Write values to config file

except KeyboardInterrupt:

    print('Stopping...')

finally:
    wheels[wheel].stop()
    print('\nStopped.')
