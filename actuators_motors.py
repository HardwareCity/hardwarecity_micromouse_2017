import constants as co

class STEPPER_42BYGHW811:
    """
    NEMA-17 Bipolar 4-wire
    Black wire = A (?) CONFIRMAR
    Green wire = B (?) CONFIRMAR
    Red Wire = C (?) CONFIRMAR
    Blue wire = D (?) CONFIRMAR
    Number of steps: 200 Step
    angle: 1.8 degrees
    Rated Voltage: 3.1V (?) CONFIRMAR
    Phase Resistance: 1.25 ohm (?) CONFIRMAR
    Rated Current : 2.5 A (?) CONFIRMAR
    Phase Inductance : 1.8 mH (?) CONFIRMAR
    Holding torque: 4.8 kg.cm (?) CONFIRMAR
    Number of Leads: 4
    Dimensions axis: 5 x 22 mm (?) CONFIRMAR
    Dimensions: 42 x 42 x 48 mm (?) CONFIRMAR
    Weight: 340 g (?) CONFIRMAR
    """
    def __init__(self, stepper_type=co.MOTOR_FULL_STEPS):
        self._degrees_by_step = 1.8
        self._step_index = 0

        steppes_full_step = [
            (True, True, False, False),
            (False, True, True, False),
            (False, False, True, True),
            (True, False, False, True)
        ]

        steppes_half_step = [
            (True, True, False, False),
            (False, True, False, False),
            (False, True, True, False),
            (False, False, True, False),
            (False, False, True, True),
            (False, False, False, True),
            (True, False, False, True),
            (True, False, False, False)
        ]

        self.stepps_table = steppes_full_step


    def move_stepps(self, direction, stepps):
        if direction == co.FRONT:
            step = 1
        else:
            step = -1


class SERVO_9g:
    def __init__(self, GPIO, pin_signal, name=''):
        self._GPIO = GPIO
        self._pin_signal = pin_signal
        self._name = name
        self._p = GPIO.PWM(self._pin_signal, 50)  # Pin X com 50Hz (20ms)

    def cleanup_gpio_pins(self):
        for pin in [self._pin_signal]:
            print "cleaning pin", pin, "from '"+self._name+"'."
            self._GPIO.cleanup(pin)

    def start(self, degrees=0):
        if degrees < -90:
            degrees = -90
        if degrees > 90:
            degrees = 90
        # dc = 0.5 / 20.0 * 100.0  # 2.5 turn towards 0 degree [-90]
        # dc = 1.5 / 20.0 * 100.0  # 7.5 turn towards 90 degree [0]
        # dc = 2.5 / 20.0 * 100.0  # 12.5 turn towards 180 degree [90]
        # User insere entre [-90 e 90], mas servo_farol funciona entre [0 e 180]
        degrees += 90
        dc = (0.5 + ((degrees*2) / 180)) / 20.0 * 100.0
        self._p.start(dc)

    def change_angle(self, degrees):
        if degrees < -90:
            degrees = -90
        if degrees > 90:
            degrees = 90
        # dc = 0.5 / 20.0 * 100.0  # 2.5 turn towards 0 degree [-90]
        # dc = 1.5 / 20.0 * 100.0  # 7.5 turn towards 90 degree [0]
        # dc = 2.5 / 20.0 * 100.0  # 12.5 turn towards 180 degree [90]
        # User insere entre [-90 e 90], mas servo_farol funciona entre [0 e 180]
        degrees += 90
        dc = (0.5 + ((degrees*2) / 180)) / 20.0 * 100.0
        self._p.ChangeDutyCycle(dc)

    def stop(self):
        self._p.stop()
