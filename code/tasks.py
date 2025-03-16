from pyb import Timer, Pin
import time
from line_sensor import LineSensor
from motor import Motor
from encoder import Encoder
from controller import Controller
from bump import BumpSensor
from bno055 import BNO055

class MasterMind:
    """
    Task to continuously read the centroid from the line sensor and update
    the shared deviation variable so that the centroid is driven to the target (4).
    """
    def __init__(self, R_v_ref, L_v_ref, mode, sensor):
        self.sensor = sensor
        self.R_v_ref = R_v_ref
        self.L_v_ref = L_v_ref
        self.mode = mode
        self.state = 0

    def run(self):
        while True:
            try:
                if self.state == 0:
                    pass
                    self.state = 1
                elif self.state == 1:
                    # Tell the robot to drive straight
                    if not self.sensor.line_detected(threshold=1):
                        self.mode.put(1)
                    else:
                        self.state = 2
                    
                elif self.state == 2:
                    # Tell the robot to follow line
                    if self.sensor.line_detected(threshold=1):
                        self.mode.put(2)
                    else:
                        self.state = 1
                
                    
                else:
                    raise ValueError("Invalid state in MasterMind")
                    
                yield self.state
            except Exception as e:
                print("Error in MasterMind:", e)
                continue


class Task_Actuate_Motors:
    """
    Task to continuously read the motor velocities and control them to match the reference velocities.
    """
    def __init__(self, R_v_ref, L_v_ref):
        # Init motors and encoders
        self.rightMotor = Motor(Timer(1, freq=1000), 3, Pin.board.PA10, Pin.board.PB8, Pin.board.PB2)
        self.leftMotor = Motor(Timer(2, freq=1000), 1, Pin.board.PA15, Pin.board.PH0, Pin.board.PH1)
        self.rightEncoder = Encoder(Timer(4, period=0xFFFF, prescaler=0), Pin.board.PB6, Pin.board.PB7)
        self.leftEncoder = Encoder(Timer(3, period=0xFFFF, prescaler=0), Pin.board.PB4, Pin.board.PB5)
        
        # Enable motors
        self.rightMotor.enable()
        self.leftMotor.enable()
        
        # Init a controller for each motor
        print("Gains right")
        self.rightController = Controller(auto_gains=False, Kp=2, Ki=2)
        # auto_gains=False, Kp=2, Ki=2
        print("Gains left")
        self.leftController = Controller(auto_gains=False, Kp=4, Ki=4)
        # auto_gains=False, Kp=4, Ki=4
        
        self.rightVel = 0
        self.leftVel = 0
        
        # The shared variables for the velocity references
        self.R_v_ref = R_v_ref
        self.L_v_ref = L_v_ref
        
        # Start in state 0
        self.state = 0

    def run(self):
        while True:
            try:
                if self.state == 0:
                    self.state = 1
                elif self.state == 1:
                    self.rightEncoder.update()
                    self.leftEncoder.update()
                    # Get velocities and convert to rad/s
                    self.rightVel = self.rightEncoder.get_velocity()
                    self.rightVel = 2 * 3.1415 * 1_000_000 * self.rightVel / 1440
                    self.leftVel = self.leftEncoder.get_velocity()
                    self.leftVel = 2 * 3.1415 * 1_000_000 * self.leftVel / 1440
                    # (Placeholder for any additional processing.)
                    self.state = 2
                elif self.state == 2:
                    # Get the current velocity reference from the shared variables.
                    rightErr = self.R_v_ref.get() - self.rightVel
                    leftErr = self.L_v_ref.get() - self.leftVel
                    
                    rightActuation = self.rightController.Control(rightErr)
                    leftActuation = self.leftController.Control(leftErr)
                    #print("Right vel:")
                    #print(self.rightVel)
                    #print("Right actuation:")
                    #print(rightActuation)
                    #print("Right position (rad)")
                    #print(2 * 3.1415 *self.rightEncoder.get_position()/1440)
                    #print("Left vel:")
                    #print(self.leftVel)
                    #print("Left actuation:")
                    #print(leftActuation)
                    #print("Left position (rad)")
                    #print(2 * 3.1415 *self.leftEncoder.get_position()/1440)

                    
                    self.rightMotor.set_effort(rightActuation, max_effort=25)
                    self.leftMotor.set_effort(leftActuation, max_effort=25)
                    self.state = 1
                else:
                    raise ValueError("Invalid state in Task_Actuate_Motors")

                yield self.state
            except Exception as e:
                print("Error in Task_Actuate_Motors:", e)
                continue
            
class Task_Drive_Straight:
    """
    Task to implement position control for driving straight.
    It uses a finite state machine with two states:
      - In state 1, it checks that the drive mode is active.
      - In state 2, it reads encoder positions, computes the error, uses the Controller
        to compute a correction, and updates the shared velocity references accordingly.
    """
    def __init__(self, R_v_ref, L_v_ref, mode, base_speed=1):
        self.R_v_ref = R_v_ref
        self.L_v_ref = L_v_ref
        self.mode = mode
        self.base_speed = base_speed
        self.state = 1
        # Create encoder instances for position measurement.
        self.rightEncoder = Encoder(Timer(4, period=0xFFFF, prescaler=0), Pin.board.PB6, Pin.board.PB7)
        self.leftEncoder = Encoder(Timer(3, period=0xFFFF, prescaler=0), Pin.board.PB4, Pin.board.PB5)
        # Instantiate a position controller using your Controller class.
        self.positionController = Controller(auto_gains=False, Kp=5, Ki=5)
        self.init_pos_left = 0
        self.init_pos_right = 0
    
    def run(self):
        while True:
            try:
                if self.state == 1:
                    # Check that the mode is set for drive-straight.
                    if self.mode.get() == 1:
                        print("Drive straight active!!!")
                        self.state = 2
                    else:
                        self.rightEncoder.update()
                        self.leftEncoder.update()
                        self.init_pos_left = 2 * 3.1415 *self.leftEncoder.get_position()/1440
                        self.init_pos_right = 2 * 3.1415 *self.rightEncoder.get_position()/1440
                elif self.state == 2:
                    # Update encoder positions.
                    
                    self.rightEncoder.update()
                    self.leftEncoder.update()
                    pos_right = 2 * 3.1415 *self.rightEncoder.get_position()/1440
                    pos_left = 2 * 3.1415 *self.leftEncoder.get_position()/1440
                    
                    d_pos_right = pos_right - self.init_pos_right
                    d_pos_left = pos_left - self.init_pos_left
                    # Compute position error (target: zero difference).
                    pos_error = d_pos_right - d_pos_left
                    # Use the Controller class to compute a correction.
                    correction = self.positionController.Control(pos_error)

                    # Set the shared velocity references.
                    #print("Drive straight right:")
                    #print(self.base_speed - correction)
                    #print("Drive straight left:")
                    #print(self.base_speed+correction)
                    self.R_v_ref.put(self.base_speed - correction)
                    self.L_v_ref.put(self.base_speed + correction)
                    
                    self.state = 1
                else:
                    raise ValueError("Invalid state in Task_Drive_Straight")
                
                yield self.state
            except Exception as e:
                print("Error in Task_Drive_Straight:", e)
                continue

            
            
class Task_Follow_Line:
    """
    Task to continuously read the centroid from the line sensor and update
    the shared deviation variable so that the centroid is driven to the target (4).
    """
    def __init__(self, R_v_ref, L_v_ref, mode, sensor, target_speed=1):
        self.sensor = sensor
        self.R_v_ref = R_v_ref
        self.L_v_ref = L_v_ref
        self.mode = mode
        # Instantiate a controller for the line sensor deviations.
        print("Gains line")
        self.lineController = Controller(auto_gains=False, Kp=1.25, Ki=.2, Kd=0)
        # auto_gains=False, Kp=.9, Ki=.15, Kd=.01
        self.target_speed = target_speed
        self.target_centroid = 4  # Desired centroid value (midpoint of 1 to 7)
        self.state = 0

    def run(self):
        while True:
            try:
                if self.state == 0:
                    if self.mode.get() == 2:
                        print("Line follow active!!!")
                        self.state = 1
                elif self.state == 1:

                    self.centroid = self.sensor.get_centroid()

                    self.state = 2
                elif self.state == 2:
                    error = self.target_centroid - self.centroid
                    actuation = self.lineController.Control(error)
                    self.R_v_ref.put(self.target_speed + actuation)
                    self.L_v_ref.put(self.target_speed - actuation)
                    self.state = 0
                else:
                    raise ValueError("Invalid state in Task_Follow_Line")
                    
                yield self.state
            except Exception as e:
                print("Error in Task_Follow_Line:", e)
                continue
            
            
class Task_IMU:
    """
    Task to initialize the IMU and continuously print the heading data.
    """
    def __init__(self):
        self.state = 0
        # Initialize IMU
        print("Initializing IMU...")
        self.imu = BNO055.initialize_imu()
        print("IMU Initialization Complete.")

    def run(self):
        while True:
            try:
                if self.state == 0:
                    self.state = 1  # Move to state 1

                elif self.state == 1:
                    # Read and print heading data indefinitely
                    heading, _, _ = self.imu.read_euler_angles()
                    print(f"Heading: {heading:.2f}°")

                else:
                    raise ValueError("Invalid state in Task_IMU")

                yield self.state  # Maintain task execution
            except Exception as e:
                print("Error in Task_IMU:", e)
                continue

