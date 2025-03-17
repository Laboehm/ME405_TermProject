from pyb import UART, Pin

class Bluetooth:
    """
    Bluetooth module driver using Serial3.

    Pin assignments:
      - PC4: Tx (nucleo)  --> Bluetooth Rx
      - PC5: Rx (nucleo)  --> Bluetooth Tx
      - PC8: Bluetooth state (input)
      - PC6: Bluetooth enable (output)
    """
    def __init__(self, baudrate=115200):
        # Configure Tx and Rx pins for UART3
        Pin('PC4', mode=Pin.ALT, alt=7)  # Set PC4 as UART3 Tx 
        Pin('PC5', mode=Pin.ALT, alt=7)  # Set PC5 as UART3 Rx 
        
        # Create a UART object for serial3
        self.uart = UART(3, baudrate)
        
        # Configure additional pins.
        self.state_pin = Pin('PC8', mode=Pin.IN)    # Bluetooth state pin
        self.enable_pin = Pin('PC6', mode=Pin.OUT_PP) # Bluetooth enable pin
        
        # Enable the Bluetooth module.
        self.enable_pin.high()
        
    def read(self):
        """Return any available data from the Bluetooth module."""
        if self.uart.any():
            return self.uart.read()
        return None
    
    def write(self, data):
        """Write data to the Bluetooth module."""
        self.uart.write(data)
    
    def echo(self):
        """
        If any data is received, read it and immediately echo it back.
        Returns the data that was echoed, or None if no data was available.
        """
        if self.uart.any():
            data = self.uart.read()
            self.uart.write(data)
            return data
        return None
