import os
import yaml

_defaultSettingsFile = 'settings.yaml'

_defaultSettings = '''
label: None

scuttle:

    chassis:

        # Wheel base (L) in meters.
        wheel_base: 0.355

        # Wheel radius in meters
        wheel_radius: 0.04165

        # Maximum Linear Velocity (m/s)
        maximum_linear_velocity: 0.45

        # Maximum Angular Velocity (rad/s)
        maximum_angular_velocity: 2.7

        # Wheel information
        wheels:

            # I2C bus id
            i2c_bus_id: 1

            # Wheel Loop Frequency
            wheel_frequency: 50

            # Motor PWM frequency in Hz
            pwm_frequency: 150

            # Open loop wheel speed control
            openloop: True

            # Left wheel details
            l_wheel:

                minimum_forward_duty: 0.22
                maximum_forward_duty: 1
                minimum_forward_angular_velocity: 1
                maximum_forward_angular_velocity: 10
                minimum_backward_duty: -0.22
                maximum_backward_duty: -1
                minimum_backward_angular_velocity: -1
                maximum_backward_angular_velocity: -10

                # minimum_matching_forward_duty: 1
                # maximum_matching_forward_duty: 1
                # minimum_matching_backward_duty: 1
                # maximum_matching_backward_duty: 1

                encoder:
                    # Encoder address
                    address: 0x40

                    # Flag to indicate if the encoder data needs to be inverted
                    invert: True

                motor:

                    # Flag to indicate if the motor duty cycle needs to be inverted
                    invert: False

                    # GPIO pin designated as digital
                    digital: 11

                    # GPIO pin designed as PWM
                    pwm: 12

            # Right wheel details
            r_wheel:

                minimum_forward_duty: 0.22
                maximum_forward_duty: 1
                minimum_forward_angular_velocity: 1
                maximum_forward_angular_velocity: 10
                minimum_backward_duty: -0.22
                maximum_backward_duty: -1
                minimum_backward_angular_velocity: -1
                maximum_backward_angular_velocity: -10

                # minimum_matching_forward_duty: 1
                # maximum_matching_forward_duty: 1
                # minimum_matching_backward_duty: 1
                # maximum_matching_backward_duty: 1

                encoder:
                    # Encoder address
                    address: 0x41

                    # Flag to indicate if the encoder data needs to be inverted
                    invert: False

                motor:

                    # Flag to indicate if the motor duty cycle needs to be inverted
                    invert: False

                    # GPIO pin designated as digital
                    digital: 15

                    # GPIO pin designed as PWM
                    pwm: 16

        # Platform controlling the robot
        # RPi - Raspberry PI
        # Beagle - Beagle
        # TISK - TI SK board
        # Nano - NVIDIA Jetson Nano board
        motor_control_platform: 'RPi'

    # Sensor specific information
    sensors: None
'''

class Settings:

    def __init__(self, file=None):

        if file is None:
            file = _defaultSettingsFile
        else:
            pass

        self.path = os.path.join(os.getcwd(), file)

        # Check if settings file exists.
        if os.path.exists(file):
            # print('Found config file', file)
            pass

        # If settings file does not exist and none is specified
        elif not os.path.exists(file) and file is _defaultSettingsFile:
            print('Could not find config file', file)
            with open(self.path, "w") as settingsFile:
                settingsFile.write(_defaultSettings)
                settingsFile.close()
                print('Created config file', file)

        # If specified settings file does not exist
        else:
            raise Exception('Config file "'+file+'" does not exist.')

        # Open settings file
        with open(self.path, "r") as settingsFile:
            # self.settings = yaml.load(settingsFile)
            settings = yaml.safe_load(os.path.expandvars(settingsFile.read()))

            # Validate the YAML specification
            # There's gotta be a more efficient way to do this.
            if 'scuttle' not in settings.keys():
                raise Exception("Cannot find 'scuttle' section in " + file)
            elif 'chassis' not in settings['scuttle'].keys():
                raise Exception("Cannot find 'chassis' section in " + file)
            elif 'wheels' not in settings['scuttle']['chassis'].keys():
                raise Exception("Cannot find 'wheels' section in " + file)
            elif 'l_wheel' not in settings['scuttle']['chassis']['wheels'].keys():
                raise Exception("Cannot find 'l_wheel' section in " + file)
            elif 'r_wheel' not in settings['scuttle']['chassis']['wheels'].keys():
                raise Exception("Cannot find 'r_wheel' section in " + file)
            elif 'motor' not in settings['scuttle']['chassis']['wheels']['l_wheel'].keys():
                raise Exception("Cannot find 'motor' section in " + file)
            elif 'motor' not in settings['scuttle']['chassis']['wheels']['r_wheel'].keys():
                raise Exception("Cannot find 'motor' section in " + file)

            # There's gotta be a more efficient way to do this too.
            chassis = settings['scuttle']['chassis']

            self.WHEEL_BASE = chassis['wheel_base']
            self.WHEEL_RADIUS = chassis['wheel_radius']

            self.I2C_BUS = chassis['wheels']['i2c_bus_id']
            self.OPENLOOP = chassis['wheels']['openloop']
            self.MOTOR_PWM_FREQUENCY = chassis['wheels']['pwm_frequency']
            self.PLATFORM = chassis['motor_control_platform']

            self.LEFT_WHEEL_MOTOR_PINS = (chassis['wheels']['l_wheel']['motor']['digital'], chassis['wheels']['l_wheel']['motor']['pwm'])
            self.LEFT_WHEEL_ENCODER_ADDRESS = chassis['wheels']['l_wheel']['encoder']['address']
            self.LEFT_WHEEL_MOTOR_INVERT = chassis['wheels']['l_wheel']['motor']['invert']
            self.LEFT_WHEEL_ENCODER_INVERT = chassis['wheels']['l_wheel']['encoder']['invert']

            self.RIGHT_WHEEL_MOTOR_PINS = (chassis['wheels']['r_wheel']['motor']['digital'], chassis['wheels']['r_wheel']['motor']['pwm'])
            self.RIGHT_WHEEL_ENCODER_ADDRESS = chassis['wheels']['r_wheel']['encoder']['address']
            self.RIGHT_WHEEL_MOTOR_INVERT = chassis['wheels']['r_wheel']['motor']['invert']
            self.RIGHT_WHEEL_ENCODER_INVERT = chassis['wheels']['r_wheel']['encoder']['invert']

    # def read(self):

    # def write(self):
