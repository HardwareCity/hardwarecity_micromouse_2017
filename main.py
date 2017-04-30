import constants as co

import RPi.GPIO as GPIO

from actuators_motor_drivers import DRV8825
from actuators_motors import STEPPER_42BYGHW811, SERVO_9g

print "GPIO.VERSION", GPIO.VERSION
# set up GPIO using BCM numbering
GPIO.setmode(GPIO.BCM)

motor_left = DRV8825(GPIO, STEPPER_42BYGHW811(stepper_type=co.MOTOR_FULL_STEPS), pin_step=2, pin_dir=3, name='Motor Esquerdo')
motor_right = DRV8825(GPIO, STEPPER_42BYGHW811(stepper_type=co.MOTOR_HALF_STEPS), pin_step=4, pin_dir=17, name='Motor Direito')

servo_farol = SERVO_9g(GPIO, 27)
servo_farol.start(0)



servo_farol.stop()
servo_farol.cleanup_gpio_pins()
motor_left.cleanup_gpio_pins()
motor_right.cleanup_gpio_pins()
