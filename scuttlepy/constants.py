I2C_BUS = 1

WHEEL_BASE = 0.1775                 # L - meters    Measured from center of wheel base to inside edge of wheel.
WHEEL_RADIUS = 0.04165              # R - meters

LEFT_ENCODER_ADDRESS = 0x43         # Left wheel encoder address
RIGHT_ENCODER_ADDRESS = 0x41        # Right wheel encoder address

LEFT_MOTOR_CHANNEL = (11,12)        # Create Left Motor Object (pwm, digital)
RIGHT_MOTOR_CHANNEL = (15,16)       # Create Right Motor Object (pwm, digital)
