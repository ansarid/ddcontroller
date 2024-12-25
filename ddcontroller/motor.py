import board
from adafruit_pca9685 import PCA9685

class Motor:

    def __init__(self, pins=[0,1], rpm=255, invert=False, pwm_frequency=100, decay='SLOW'):

        self.pins = pins

        self.rpm = rpm

        self.decay = decay

        # Initial Duty %
        self.duty = 0

        self.max_duty = 1
        self.min_duty = -1

        # PWM frequency (Hz)
        self.pwm_frequency = pwm_frequency

        # Reverse motor direction
        self.invert = invert

        self.i2c = board.I2C()

        self.pca = PCA9685(self.i2c)
        self.pca.frequency = self.pwm_frequency

        for pin in self.pins:
            self.pca.channels[pin].duty_cycle = int((0xFFFF)*(self.duty/100))

    def set_pwm_frequency(self, frequency):
        self.pwm_frequency = frequency
        self.pca.frequency = self.pwm_frequency

    def set_duty(self, duty):
        duty = round(
            sorted((-1, float(duty), 1))[1], 2
        )

        self.duty = duty

        if self.decay == 'FAST':

            if duty == 0:
                for pin in self.pins:
                    self.pca.channels[pin].duty_cycle = 0x0

            elif duty > 0:
                self.pca.channels[self.pins[0]].duty_cycle = int((2**16)*duty)
                self.pca.channels[self.pins[1]].duty_cycle = 0x0

            elif duty < 0:
                self.pca.channels[self.pins[0]].duty_cycle = 0x0
                self.pca.channels[self.pins[1]].duty_cycle = -int((2**16)*duty)


        if self.decay == 'SLOW':

            if duty == 0:
                for pin in self.pins:
                    self.pca.channels[pin].duty_cycle = 0xFFFF

            elif duty > 0:
                self.pca.channels[self.pins[0]].duty_cycle = 0xFFFF
                self.pca.channels[self.pins[1]].duty_cycle = (2**16) - int((2**16)*duty)

            elif duty < 0:
                self.pca.channels[self.pins[0]].duty_cycle = (2**16) - int((2**16)*abs(duty))
                self.pca.channels[self.pins[1]].duty_cycle = 0xFFFF


    def stop(self):
        for pin in self.pins:
            self.pca.channels[pin].duty_cycle = 0x0


if __name__== '__main__':

    import time

    motor = Motor(pins=[0,1], rpm=255, decay='SLOW') # 255rpm @ 6V
    motor.set_duty(-0.2)
    time.sleep(5)
    motor.set_duty(0.0)