from pyb import Pin, ADC
import time

class LineSensor:
    """
    Hardware driver for a 7-sensor line sensor array on the Nucleo L476RG.

    Pin assignments:
      - Emitter control:
          - Even sensors: PC15
          - Odd sensors: PC14
      - Sensor read channels:
          - Sensor 1: PC3
          - Sensor 2: PA4
          - Sensor 3: PA0
          - Sensor 4: PB0
          - Sensor 5: PA1
          - Sensor 6: PC1
          - Sensor 7: PC0
    
    Calibration:
      When instantiated, the sensor will automatically calibrate unless you pass
      calibrate_on_init=False. Calibration is blocking code that waits for
      the user to place a white surface, then a black surface, and press Enter.
    """
    def __init__(self, calibrate_on_init=True):
        # Set up emitter control pins (current supply for emitters)
        self.emitter_even = Pin('PC15', mode=Pin.OUT_PP, value=0)
        self.emitter_odd = Pin('PC14', mode=Pin.OUT_PP, value=0)
        
        # Set up ADC channels for sensor readings (assumed analog sensors)
        self.sensor1 = ADC(Pin('PC3'))
        self.sensor2 = ADC(Pin('PA4'))
        self.sensor3 = ADC(Pin('PA0'))
        self.sensor4 = ADC(Pin('PB0'))
        self.sensor5 = ADC(Pin('PA1'))
        self.sensor6 = ADC(Pin('PC1'))
        self.sensor7 = ADC(Pin('PC0'))
        
        # Calibration values will be stored here after calibration.
        self.white_calibration = None
        self.black_calibration = None
        
        # Optional automatic calibration upon instantiation.
        if calibrate_on_init:
            self.calibrate()

    def enable_emitters(self):
        """Enable both even and odd sensor emitters."""
        self.emitter_even.high()
        self.emitter_odd.high()
    
    def disable_emitters(self):
        """Disable both sensor emitters."""
        self.emitter_even.low()
        self.emitter_odd.low()
    
    def read_sensors(self):
        """
        Read all sensor values.
        
        Returns:
            A dictionary with sensor numbers (1-7) as keys and ADC values as values.
        """
        self.enable_emitters()
        output = {
            1: self.sensor1.read(),
            2: self.sensor2.read(),
            3: self.sensor3.read(),
            4: self.sensor4.read(),
            5: self.sensor5.read(),
            6: self.sensor6.read(),
            7: self.sensor7.read()
        }
        self.disable_emitters()
        return output
    
    def calibrate(self):
        """
        Calibrate the sensor bias for white and black surfaces.
        
        Blocking method:
          1. Prompts the user to place a white surface under the sensors.
          2. Waits for the user to press Enter.
          3. Records the white calibration values.
          4. Prompts the user to place a black surface under the sensors.
          5. Waits for the user to press Enter.
          6. Records the black calibration values.
        """
        def wait_for_enter(prompt):
            """Helper function that repeatedly tries to read input until successful."""
            print(prompt)
            while True:
                try:
                    input()  # Wait for Enter
                    break
                except EOFError:
                    time.sleep(0.1)  # Wait briefly and try again

        # Ensure the emitters are enabled during calibration.
        self.enable_emitters()
        
        # Calibrate for white surface.
        print("Place a white surface under the sensor array.")
        wait_for_enter("Press Enter when ready for white calibration...")
        white = self.read_sensors()
        print("White calibration values:", white)
        
        # Calibrate for black surface.
        print("Now place a black surface under the sensor array.")
        wait_for_enter("Press Enter when ready for black calibration...")
        black = self.read_sensors()
        print("Black calibration values:", black)
        
        # Store the calibration values.
        self.white_calibration = white
        self.black_calibration = black
        
        print("Calibration complete.")
    
    def get_centroid(self):
        """
        Read the current sensor values, normalize them using calibration,
        and compute the centroid of the sensor array.

        Each sensor's normalized value is computed as:
            normalized = (raw_value - white) / (black - white)

        The centroid is calculated as the weighted average of sensor indices,
        where each sensor's weight is its normalized reading.
        """
        readings = self.read_sensors()
        values = []
        
        # Collect sensor readings in order (sensor 1 to 7)
        for sensor in sorted(readings.keys()):
            raw = readings[sensor]
            # Normalize if calibration values are available
            if self.white_calibration is not None and self.black_calibration is not None:
                white = self.white_calibration[sensor]
                black = self.black_calibration[sensor]
                denom = (black - white) if (black - white) != 0 else 1
                normalized = (raw - white) / denom
            else:
                normalized = raw
            values.append(normalized)
        
        total_weighted = sum(values[i] * (i + 1) for i in range(7))
        total = sum(values)
        centroid = total_weighted / total if total != 0 else 0
        if centroid > 7:
            centroid = 7
        elif centroid < 1:
            centroid = 1
        return centroid

    def line_detected(self, threshold=1):
        """
        Determine if a line is detected by summing normalized sensor values.
        
        Each sensor's normalized value is computed as:
            normalized = (raw_value - white) / (black - white)
        If the sum of these normalized values exceeds the threshold, a line is detected.

        Returns bool: True if a line is detected, False otherwise.
        """
        readings = self.read_sensors()
        total = 0
        for sensor in sorted(readings.keys()):
            raw = readings[sensor]
            if self.white_calibration is not None and self.black_calibration is not None:
                white = self.white_calibration[sensor]
                black = self.black_calibration[sensor]
                denom = (black - white) if (black - white) != 0 else 1
                normalized = (raw - white) / denom
            else:
                normalized = raw
            total += normalized
        
        return total > threshold
