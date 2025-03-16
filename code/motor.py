from pyb import Pin, Timer

class Motor:
    def __init__(self, PWMtimer, PWM_channel, PWM, DIR, nSLP):
        self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)
        self.PWM_pin = PWMtimer.channel(PWM_channel, Timer.PWM, pin=PWM)
        self.DIR_pin = Pin(DIR, mode=Pin.OUT_PP, value=0)

    def set_effort(self, effort, max_effort=100):
        # Ensure max_effort does not exceed 100
        max_effort = min(100, max_effort)

        # Scale effort to respect max_effort
        effort = max(-max_effort, min(max_effort, effort))

        if effort >= 0:
            self.DIR_pin.value(0)
            self.PWM_pin.pulse_width_percent(effort)
        else:
            self.DIR_pin.value(1)
            self.PWM_pin.pulse_width_percent(abs(effort))

    def enable(self):
        self.nSLP_pin.high()  # Enable motor
        self.DIR_pin.value(1)
        self.PWM_pin.pulse_width_percent(0)

    def disable(self):
        self.nSLP_pin.low()  # Disable motor
