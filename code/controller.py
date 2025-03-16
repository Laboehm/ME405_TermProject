from time import ticks_us, ticks_diff

class Controller:
    def __init__(self, auto_gains=True, Kp=0.0, Ki=0.0, Kd=0.0):
        """
        Initialize the Controller with PID gains.

        Args:
            auto_gains (bool): If True, prompt the user to input Kp, Ki, and Kd.
                               If False, use the provided Kp, Ki, and Kd values.
            Kp (float): Proportional gain (used if auto_gains is False).
            Ki (float): Integral gain (used if auto_gains is False).
            Kd (float): Derivative gain (used if auto_gains is False).
        """
        if auto_gains:
            self.Kp = float(input("Enter Kp: "))
            self.Ki = float(input("Enter Ki: "))
            self.Kd = float(input("Enter Kd: "))
        else:
            self.Kp = Kp
            self.Ki = Ki
            self.Kd = Kd

        self.integral = 0.0
        self.last_val = 0.0  # Stores the last error value for derivative calculation
        self.time_step = 0.0 # Most recent time step in seconds
        self.last_time = ticks_us()  # Initialize time with current tick count

    def Control(self, error):
        """
        Compute the PID control signal based on the error input.

        This method calculates the time step since the last call using ticks and ticks_diff,
        updates the integral and derivative, and returns the actuation signal.
        """
        current_time = ticks_us()
        # Compute dt in seconds
        dt = ticks_diff(current_time, self.last_time) / 1e6
        self.time_step = dt
        self.last_time = current_time

        # Update integral
        self.integral += error * dt

        # Compute derivative term
        derivative = (error - self.last_val) / dt if dt > 0 else 0.0

        # PID control signal.
        control_signal = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Update last error value.
        self.last_val = error

        return control_signal
