import time
import numpy as np
from ddcontroller.wheels import Wheel

# Create right wheel object
right_wheel = Wheel(digital_pin=15,
              pwm_pin=16,
              pwm_frequency=150,
              i2c_bus=1,
              encoder_address=0x41,
              wheel_radius=0.04165,
              motor_pulley_teeth=15,
              wheel_pulley_teeth=30,
              )

left_wheel  = Wheel(digital_pin=11,
              pwm_pin=12,
              pwm_frequency=150,
              i2c_bus=1,
              encoder_address=0x40,
              wheel_radius=0.04165,
              motor_pulley_teeth=15,
              wheel_pulley_teeth=30,
              )

try:

    # Create infinite loop
    while True:

        # Update wheel measurements
        left_wheel.update()
        right_wheel.update()

        # Set wheel angular velocity to 2*pi
        left_wheel.motor.set_duty(1)
        right_wheel.motor.set_duty(1)

        print((2*(left_wheel.get_angular_velocity()*60)/(np.pi*2)), (2*(right_wheel.get_angular_velocity()*60)/(np.pi*2)))

        # Run loop at 50Hz
        time.sleep(1/50)

except KeyboardInterrupt:

    print('Stopping...')

finally:
    # Clean up.
    left_wheel.stop()
    right_wheel.stop()
    print('Stopped.')