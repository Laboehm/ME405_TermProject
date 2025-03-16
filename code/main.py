import cotask
import task_share
from tasks import MasterMind, Task_Actuate_Motors, Task_Drive_Straight, Task_Follow_Line, Task_IMU
from pyb import Timer, Pin
from line_sensor import LineSensor

def main():
    R_v_ref = task_share.Share('f', name="Right reference velocity")
    L_v_ref = task_share.Share('f', name="Left reference velocity")
    mode = task_share.Share('I', name="Drive mode")
    mode.put(2)
    
    try:
        # Instantiate the actuate motor task.
        actuate_motor_obj = Task_Actuate_Motors(R_v_ref, L_v_ref)
        actuate_motor_task = cotask.Task(actuate_motor_obj.run,
                                         name="Motor Control Task",
                                         priority=3,
                                         period=30,
                                         profile=True,
                                         trace=True)
        cotask.task_list.append(actuate_motor_task)
        
        actuate_motor_obj.rightMotor.disable()
        actuate_motor_obj.leftMotor.disable()
        
        sensor = LineSensor()
        
        # Instantiate the master mind task.
        mastermind_obj = MasterMind(R_v_ref, L_v_ref, mode, sensor)
        mastermind_task = cotask.Task(mastermind_obj.run,
                                      name="MasterMind Task",
                                      priority=3,
                                      period=10,
                                      profile=True,
                                      trace=True)
        cotask.task_list.append(mastermind_task)
        
        # Instantiate the drive straight task.
        drive_straight_obj = Task_Drive_Straight(R_v_ref, L_v_ref, mode, 3)
        drive_straight_task = cotask.Task(drive_straight_obj.run,
                                          name="Drive Straight Task",
                                          priority=3,
                                          period=10,
                                          profile=True,
                                          trace=True)
        cotask.task_list.append(drive_straight_task)
        
        # Instantiate the follow line task.
        follow_line_obj = Task_Follow_Line(R_v_ref, L_v_ref, mode, sensor, 3)
        follow_line_task = cotask.Task(follow_line_obj.run,
                                       name="Follow Line Task",
                                       priority=3,
                                       period=10,
                                       profile=True,
                                       trace=True)
        cotask.task_list.append(follow_line_task)

        # Instantiate the IMU task.
        imu_task_obj = Task_IMU()
        imu_task = cotask.Task(imu_task_obj.run,
                               name="IMU Task",
                               priority=3,
                               period=100,
                               profile=True,
                               trace=True)
        cotask.task_list.append(imu_task)

        actuate_motor_obj.rightMotor.enable()
        actuate_motor_obj.leftMotor.enable()
        
        print("Press Enter to start")
        input()
        
        # Run the scheduler.
        while True:
            cotask.task_list.pri_sched()
            
    except KeyboardInterrupt:
        actuate_motor_obj.rightMotor.disable()
        actuate_motor_obj.leftMotor.disable()
        
        print(cotask.task_list)
        print(actuate_motor_task.get_trace())
        print(drive_straight_task.get_trace())
        print(imu_task.get_trace())
        print("Exiting")
    except Exception as e:
        # Catch any other exceptions and print the type and the exception message.
        print("Exception type:", type(e))
        print("Exception:", e)

if __name__ == "__main__":
    main()
