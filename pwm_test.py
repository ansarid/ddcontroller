import time
import RPi.GPIO as io
io.setmode(io.BOARD)

pwm_r_pin = 11	# Right Motors PWM
pwm_l_pin = 15	# Left Motors PWM

DC_l = 0 	# Duty Cycle in % from (0-100) - Left Motors pwm
DC_r = 0	# Duty Cycle in % from (0-100) - Right Motors pwm
io.setup(pwm_l_pin, io.OUT)
io.setup(pwm_r_pin, io.OUT)

io.output(pwm_l_pin, io.LOW)
io.output(pwm_r_pin, io.LOW)

pwm_l = io.PWM(pwm_l_pin, 25000)     # set Frequecy to 1KHz
pwm_r = io.PWM(pwm_r_pin, 25000)     # set Frequecy to 1KHz

pwm_l.start(0)                     # Start PWM output, Duty Cycle = 0
pwm_r.start(0)                     # Start PWM output, Duty Cycle = 0

duty_l = 0
duty_r = 0


while True:

    print('ON')
    pwm_l.ChangeDutyCycle(100)     # Change duty cycle
    pwm_r.ChangeDutyCycle(100)     # Change duty cycle

    time.sleep(5)

    print('OFF')
    pwm_l.ChangeDutyCycle(0)     # Change duty cycle
    pwm_r.ChangeDutyCycle(0)     # Change duty cycle

    time.sleep(5)
