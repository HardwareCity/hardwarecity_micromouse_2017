import constants as co

import RPi.GPIO as GPIO

from actuators_motor_drivers import DRV8825
from actuators_motors import STEPPER_42BYGHW811

print "GPIO.VERSION", GPIO.VERSION
# set up GPIO using BCM numbering
GPIO.setmode(GPIO.BCM)

motor_left = DRV8825(GPIO, STEPPER_42BYGHW811(stepper_type=co.MOTOR_FULL_STEPS), pin_step=2, pin_dir=3)
motor_right = DRV8825(GPIO, STEPPER_42BYGHW811(stepper_type=co.MOTOR_HALF_STEPS), pin_step=4, pin_dir=17)

motor_left.cleanup_gpio_pins()