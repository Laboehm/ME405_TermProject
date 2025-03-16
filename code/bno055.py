import pyb
import time
import struct

class BNO055:
    BNO055_ADDR = 0x28  # Default I2C address for BNO055
    OPR_MODE_REG = 0x3D
    CALIB_STAT_REG = 0x35
    EULER_H_LSB = 0x1A
    GYRO_DATA_X_LSB = 0x14
    CALIB_DATA_START = 0x55
    PAGE_ID_REG = 0x07

    # Operating modes
    CONFIG_MODE = 0x00
    NDOF_MODE = 0x0C  # Full sensor fusion mode

    def __init__(self, i2c, rst_pin):
        """ Initializes the BNO055 using pyb.I2C and a reset pin. """
        self.i2c = i2c
        self.rst = pyb.Pin(rst_pin, pyb.Pin.OUT_PP)  # Configure reset pin as push-pull output
        
        # Reset the BNO055
        self.reset()
        
        self.set_mode(self.CONFIG_MODE)  # Start in configuration mode
        time.sleep(0.05)
        self.set_mode(self.NDOF_MODE)  # Switch to NDOF mode
        time.sleep(0.05)

    @staticmethod
    def initialize_imu():
        """
        Initialize I2C and IMU sensor.
        """
        print("Configuring PB13 & PB14 for I2C2...")

        # Use string-based pin names
        pyb.Pin("PB13", mode=pyb.Pin.ALT, alt=4)  # SCL
        pyb.Pin("PB14", mode=pyb.Pin.ALT, alt=4)  # SDA

        print("Initializing I2C2 for BNO055...")
        i2c = pyb.I2C(2, pyb.I2C.CONTROLLER, baudrate=400000)
        time.sleep(0.1)

        print("Initializing IMU (BNO055)...")
        imu = BNO055(i2c, rst_pin=pyb.Pin("PA2", mode=pyb.Pin.OUT_PP))  # Fixed reset pin
        return imu

    def reset(self):
        """ Resets the BNO055 using the hardware reset pin. """
        self.rst.low()
        time.sleep(0.1)
        self.rst.high()
        time.sleep(0.7)  # Wait for reset to complete

    def set_mode(self, mode):
        """ Sets the operating mode of the BNO055. """
        self.i2c.mem_write(mode, self.BNO055_ADDR, self.OPR_MODE_REG)
        time.sleep(0.03)

    def get_calibration_status(self):
        """ Reads and returns the calibration status (0-3) for system, gyro, accel, and mag. """
        status = self.i2c.mem_read(1, self.BNO055_ADDR, self.CALIB_STAT_REG)
        return status[0] if status else 0

    def read_euler_angles(self):
        """ Reads Euler angles (Heading, Roll, Pitch) from the BNO055. """
        data = self.i2c.mem_read(6, self.BNO055_ADDR, self.EULER_H_LSB)
        heading, roll, pitch = struct.unpack('<hhh', data)
        return heading / 16.0, roll / 16.0, pitch / 16.0  # Convert to degrees

    def read_angular_velocity(self):
        """ Reads angular velocity (X, Y, Z) from the gyroscope in dps (degrees per second). """
        data = self.i2c.mem_read(6, self.BNO055_ADDR, self.GYRO_DATA_X_LSB)
        x, y, z = struct.unpack('<hhh', data)
        return x / 16.0, y / 16.0, z / 16.0  # Convert to dps

    def get_calibration_data(self):
        """ Retrieves calibration coefficients from the BNO055. """
        self.set_mode(self.CONFIG_MODE)  # Enter configuration mode
        data = self.i2c.mem_read(22, self.BNO055_ADDR, self.CALIB_DATA_START)
        self.set_mode(self.NDOF_MODE)  # Return to normal mode
        return data

    def set_calibration_data(self, data):
        """ Writes calibration coefficients back to the BNO055. """
        self.set_mode(self.CONFIG_MODE)
        self.i2c.mem_write(data, self.BNO055_ADDR, self.CALIB_DATA_START)
        self.set_mode(self.NDOF_MODE)

    def save_calibration_to_file(self, filename="calibration.txt"):
        """ Saves calibration coefficients to a file on MicroPython's filesystem. """
        data = self.get_calibration_data()
        with open(filename, "wb") as f:
            f.write(data)

    def load_calibration_from_file(self, filename="calibration.txt"):
        """ Loads calibration coefficients from a file. """
        try:
            with open(filename, "rb") as f:
                data = f.read()
            if len(data) == 22:
                self.set_calibration_data(data)
                return True
        except OSError:
            pass
        return False
