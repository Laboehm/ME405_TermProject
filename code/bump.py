from pyb import Pin

class BumpSensor:
    """
    Hardware driver for a bump sensor on the Nucleo L476RG.
    Uses an interrupt to detect when the sensor is triggered.
    
    Args:
        pin (str): The pin to which the bump sensor is connected.
        callback (function): Function to call when the sensor is triggered.
    """
    def __init__(self, pin, callback):
        self.pin = Pin(pin, mode=Pin.IN, pull=Pin.PULL_UP)
        self.callback = callback
        
        # Configure interrupt on falling edge (assuming active-low switch)
        self.pin.irq(trigger=Pin.IRQ_FALLING, handler=self._handle_interrupt)
    
    def _handle_interrupt(self, pin):
        """Internal method to handle the interrupt and call the user callback."""
        if self.callback:
            self.callback()
    
    def read(self):
        """Returns the current state of the bump sensor (0 = triggered, 1 = not triggered)."""
        return self.pin.value()