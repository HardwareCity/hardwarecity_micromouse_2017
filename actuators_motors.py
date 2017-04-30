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




