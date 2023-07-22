from controller import Keyboard, Robot
from simple_pid import PID
from math import pi, pow
import sys
try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found.")


def CLAMP(value, value_min, value_max):
    return min(max(value, value_min), value_max)


if __name__ == "__main__":
    robot = Robot()

    TIME_STEP = 32

    # Get and enable devices.
    camera = robot.getDevice("camera")
    camera.enable(TIME_STEP)
    
    compass = robot.getDevice("compass")
    compass.enable(TIME_STEP)

    imu = robot.getDevice("inertial unit")
    imu.enable(TIME_STEP)

    gps = robot.getDevice("gps")
    gps.enable(TIME_STEP)

    gyro = robot.getDevice("gyro")
    gyro.enable(TIME_STEP)
    
    front_left_led = robot.getDevice("front left led")
    front_right_led = robot.getDevice("front right led")  

    front_left_motor = robot.getDevice("front left propeller")
    front_right_motor = robot.getDevice("front right propeller")
    rear_left_motor = robot.getDevice("rear left propeller")
    rear_right_motor = robot.getDevice("rear right propeller")

    camera_roll_motor = robot.getDevice("camera roll")
    camera_pitch_motor = robot.getDevice("camera pitch")

    motors = [front_left_motor, front_right_motor,
              rear_left_motor, rear_right_motor]
          
    for motor in motors:
        motor.setPosition(float('inf'))
        motor.setVelocity(1)
        
    print("Start the drone")
    
    while robot.step(TIME_STEP) != -1:
        if robot.getTime() > 1:
            break
            
    k_vertical_thrust = 68.5
    k_vertical_offset = 0.6
    
    k_vertical_p = 3.0
    k_roll_p = 50.0
    k_pitch_p = 30.0
    
    target_altitude = 1.0
    
    while robot.step(TIME_STEP) != -1:
        time = robot.getTime()
        
        roll = imu.getRollPitchYaw()[0]
        pitch = imu.getRollPitchYaw()[1]

        altitude = gps.getValues()[2]       
        roll_velocity, pitch_velocity, _ = gyro.getValues()
        
        led_state: bool = int(time) % 2
        
        front_left_led.set(led_state)
        front_right_led.set(not led_state)
        
        camera_roll_motor.setPosition(-0.115 * roll_velocity)
        camera_pitch_motor.setPosition(-0.1 * pitch_velocity)
        
        roll_disturbance = 0.0
        pitch_disturbance = 0.0
        yaw_disturbance = 0.0
        
        roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance
        pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance
        yaw_input = yaw_disturbance
        clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0)
        vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0)

        front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
        front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
        rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
        rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input

        front_left_motor.setVelocity(front_left_motor_input)
        front_right_motor.setVelocity(-front_right_motor_input)
        rear_left_motor.setVelocity(-rear_left_motor_input)
        rear_right_motor.setVelocity(rear_right_motor_input)
