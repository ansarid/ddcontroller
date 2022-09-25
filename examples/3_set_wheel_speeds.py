import time
import numpy as np
from ddcontroller.wheels import Wheel

# Create right wheel object
wheel = Wheel(digital_pin=11,
              pwm_pin=12,
              pwm_frequency=150,
              i2c_bus=1,
              encoder_address=0x40,
              wheel_radius=0.04165,
              motor_pulley_teeth=13,
              wheel_pulley_teeth=25,
              )

try:

    # Create infinite loop
    while True:

        # Update wheel measurements
        wheel.update()

        # Print current wheel angular velocity
        print('{}'.format(wheel.get_angular_velocity()))

        # Set wheel angular velocity to 2*pi
        wheel.set_angular_velocity(2*np.pi)

        # Run loop at 50Hz
        time.sleep(1/50)

except KeyboardInterrupt:

    print('Stopping...')

finally:
    # Clean up.
    wheel.stop()
    print('Stopped.')