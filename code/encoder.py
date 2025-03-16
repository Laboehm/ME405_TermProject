from pyb import Timer
from time import ticks_us, ticks_diff

class Encoder:
    def __init__(self, tim, chA_pin, chB_pin):
        self.tim_pin = tim
        self.chA_pin = self.tim_pin.channel(1, pin=chA_pin, mode=Timer.ENC_AB)
        self.chB_pin = self.tim_pin.channel(2, pin=chB_pin, mode=Timer.ENC_AB)
        self.position = 0 
        self.prev_count = 0 
        self.current_count = 0
        self.delta = 0 
        self.prev_tick = ticks_us()
        self.curr_tick = 0
        self.dt = 0 
    
    def update(self):
        self.current_count = self.tim_pin.counter()
        self.curr_tick = ticks_us()
        self.delta = self.current_count - self.prev_count
        self.dt = ticks_diff(self.curr_tick, self.prev_tick)
        
        # Overflow correction
        if self.delta >= (65535 + 1) / 2:
            self.delta -= 65535 + 1
        elif self.delta < -(65535 + 1) / 2:
            self.delta += 65535 + 1
        
        self.position += self.delta
        self.prev_tick = self.curr_tick
        self.prev_count = self.current_count
    
    def get_position(self):
        return -self.position
    
    def get_velocity(self):
        return -self.delta / self.dt if self.dt else 0
    
    def zero(self):
        self.position = 0
