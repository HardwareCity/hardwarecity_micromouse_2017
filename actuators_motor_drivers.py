import constants as co

class DRV8825:
    def __init__(self, GPIO, stepper, pin_step, pin_dir, direction_inverted=False, stepps_by_meter=None):
        self._GPIO = GPIO
        self._stepper = stepper
        self._pin_step = pin_step
        self._pin_dir = pin_dir
        self._direction_inverted = direction_inverted
        self._stepps_by_meter = stepps_by_meter

        GPIO.setup(pin_step, self._GPIO.OUT)
        GPIO.setup(pin_dir, self._GPIO.OUT)

    def move_stepps(self, direction_front, stepps):
        if self._direction_inverted:
            direction_front = not direction_front
        if direction_front == co.FRONT:
            step = 1
        else:
            step = -1




