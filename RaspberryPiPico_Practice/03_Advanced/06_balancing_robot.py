"""
Raspberry Pi Pico - Advanced Program 6: PID-Controlled Balancing Robot
=====================================================================

This program demonstrates how to implement a PID control algorithm to maintain
balance of a two-wheeled robot, similar to a Segway or self-balancing robot.

Hardware required:
- Raspberry Pi Pico
- MPU6050 accelerometer/gyroscope module
- L298N or TB6612FNG motor driver
- 2 DC motors with wheels
- Battery pack (7.4V-12V recommended)
- Chassis for the robot (needs to be tall and narrow for balancing)
- Breadboard and jumper wires

Connections:
- MPU6050:
  - Connect VCC to 3.3V
  - Connect GND to GND
  - Connect SCL to GPIO pin 17 (I2C1 SCL)
  - Connect SDA to GPIO pin 16 (I2C1 SDA)

- Motor Driver (L298N example):
  - Connect ENA to GPIO pin 6 (PWM)
  - Connect IN1 to GPIO pin 7
  - Connect IN2 to GPIO pin 8
  - Connect IN3 to GPIO pin 9
  - Connect IN4 to GPIO pin 10
  - Connect ENB to GPIO pin 11 (PWM)
  - Connect motor power to external battery
  - Connect GND to common ground

Note: You'll need to install the following libraries:
- mpu6050.py for the accelerometer/gyroscope

Instructions:
1. Set up the robot chassis with the motors and wheels
2. Mount the Pico, MPU6050, and motor driver on the chassis
3. Connect the components as described above
4. Upload the required libraries to your Pico
5. Upload this program to the Pico
6. Place the robot upright and power it on
7. The robot will attempt to balance itself

Concepts covered:
- PID control algorithm
- Sensor fusion with complementary filter
- Motor control with PWM
- Inertial Measurement Unit (IMU) data processing
- Real-time control systems
- Autonomous balancing
"""

import machine
import time
import math
import utime

# Import required libraries
try:
    import mpu6050
except ImportError:
    print("Error: Required libraries not found")
    print("Please upload mpu6050.py to your Pico")
    raise

# Set up I2C for the MPU6050
i2c = machine.I2C(1, sda=machine.Pin(16), scl=machine.Pin(17), freq=400000)

# Set up the MPU6050
try:
    imu = mpu6050.MPU6050(i2c)
    print("MPU6050 initialized")
except Exception as e:
    print(f"Error initializing MPU6050: {e}")
    raise

# Set up the motor driver pins
# Motor A - Left motor
motor_a_pwm = machine.PWM(machine.Pin(6))
motor_a_in1 = machine.Pin(7, machine.Pin.OUT)
motor_a_in2 = machine.Pin(8, machine.Pin.OUT)

# Motor B - Right motor
motor_b_pwm = machine.PWM(machine.Pin(11))
motor_b_in1 = machine.Pin(9, machine.Pin.OUT)
motor_b_in2 = machine.Pin(10, machine.Pin.OUT)

# Set PWM frequency
motor_a_pwm.freq(1000)
motor_b_pwm.freq(1000)

# Constants for the PID controller
# These values will need to be tuned for your specific robot
KP = 15.0  # Proportional gain
KI = 1.0   # Integral gain
KD = 0.5   # Derivative gain

# Constants for the complementary filter
ALPHA = 0.98  # Weight for gyroscope data

# Constants for the robot
TARGET_ANGLE = 0.0  # Target angle (0 = upright)
MAX_MOTOR_SPEED = 65535  # Maximum motor speed (16-bit PWM)
SAMPLE_TIME = 0.01  # Sample time in seconds (10ms)
MAX_ANGLE = 30.0  # Maximum angle before giving up (degrees)

# Global variables
angle = 0.0  # Current angle
last_angle = 0.0  # Previous angle
integral = 0.0  # Integral term for PID
last_error = 0.0  # Previous error for derivative term
last_time = 0  # Last sample time

# Function to read the IMU and calculate the angle
def read_imu():
    global angle, last_time
    
    try:
        # Read accelerometer and gyroscope data
        accel_data = imu.get_accel_data()
        gyro_data = imu.get_gyro_data()
        
        # Calculate angle from accelerometer data
        # atan2(y, x) gives angle in radians, convert to degrees
        accel_angle = math.atan2(accel_data['y'], accel_data['z']) * 180 / math.pi
        
        # Get gyroscope rate (degrees per second)
        gyro_rate = gyro_data['x']
        
        # Get current time
        current_time = utime.ticks_ms()
        
        # Calculate time difference in seconds
        dt = utime.ticks_diff(current_time, last_time) / 1000.0
        last_time = current_time
        
        # Prevent division by zero or unrealistic dt values
        if dt <= 0 or dt > 1.0:
            dt = SAMPLE_TIME
        
        # Calculate angle from gyroscope data
        gyro_angle = angle + gyro_rate * dt
        
        # Combine accelerometer and gyroscope data using complementary filter
        # This helps reduce noise and drift
        angle = ALPHA * gyro_angle + (1 - ALPHA) * accel_angle
        
        return angle
    
    except Exception as e:
        print(f"Error reading IMU: {e}")
        return angle  # Return the last angle if there's an error

# Function to control the motors
def set_motors(left_speed, right_speed):
    try:
        # Ensure speeds are within range (-100 to 100)
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
        # Set motor directions
        if left_speed >= 0:
            motor_a_in1.value(1)
            motor_a_in2.value(0)
        else:
            motor_a_in1.value(0)
            motor_a_in2.value(1)
            left_speed = -left_speed
        
        if right_speed >= 0:
            motor_b_in1.value(1)
            motor_b_in2.value(0)
        else:
            motor_b_in1.value(0)
            motor_b_in2.value(1)
            right_speed = -right_speed
        
        # Convert speed percentage to PWM duty cycle (0-65535)
        left_duty = int((left_speed / 100) * MAX_MOTOR_SPEED)
        right_duty = int((right_speed / 100) * MAX_MOTOR_SPEED)
        
        # Set motor speeds
        motor_a_pwm.duty_u16(left_duty)
        motor_b_pwm.duty_u16(right_duty)
    
    except Exception as e:
        print(f"Error controlling motors: {e}")
        stop_motors()

# Function to stop all motors
def stop_motors():
    motor_a_in1.value(0)
    motor_a_in2.value(0)
    motor_b_in1.value(0)
    motor_b_in2.value(0)
    motor_a_pwm.duty_u16(0)
    motor_b_pwm.duty_u16(0)

# Function to calculate PID output
def calculate_pid(angle):
    global integral, last_error
    
    # Calculate error
    error = TARGET_ANGLE - angle
    
    # Calculate integral term with anti-windup
    integral += error * SAMPLE_TIME
    integral = max(-100, min(100, integral))  # Limit integral to prevent windup
    
    # Calculate derivative term
    derivative = (error - last_error) / SAMPLE_TIME
    last_error = error
    
    # Calculate PID output
    output = KP * error + KI * integral + KD * derivative
    
    # Limit output to motor speed range
    output = max(-100, min(100, output))
    
    return output

# Function to calibrate the IMU
def calibrate_imu():
    print("Calibrating IMU...")
    print("Please keep the robot still and upright")
    
    # Wait for the robot to be placed upright
    time.sleep(2)
    
    # Take multiple readings to stabilize
    for i in range(100):
        read_imu()
        time.sleep(0.01)
    
    print("Calibration complete")

# Main balancing function
def balance_robot():
    global angle, last_angle, integral, last_error, last_time
    
    # Initialize variables
    angle = 0.0
    last_angle = 0.0
    integral = 0.0
    last_error = 0.0
    last_time = utime.ticks_ms()
    
    # Calibrate the IMU
    calibrate_imu()
    
    print("Starting balancing control loop")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            # Read the current angle
            angle = read_imu()
            
            # Check if the robot has fallen over
            if abs(angle) > MAX_ANGLE:
                print(f"Robot has fallen over! Angle: {angle:.2f} degrees")
                stop_motors()
                break
            
            # Calculate PID output
            pid_output = calculate_pid(angle)
            
            # Set motor speeds based on PID output
            # Both motors get the same speed for balancing
            # For steering, you would add/subtract a value from each motor
            set_motors(pid_output, pid_output)
            
            # Print debug information
            print(f"Angle: {angle:.2f}°, PID Output: {pid_output:.2f}")
            
            # Wait for next sample
            time.sleep(SAMPLE_TIME)
    
    except KeyboardInterrupt:
        print("\nBalancing stopped by user")
    finally:
        # Stop motors when done
        stop_motors()
        print("Motors stopped")

# Function to test motors
def test_motors():
    print("Testing motors...")
    
    # Test left motor forward
    print("Left motor forward")
    motor_a_in1.value(1)
    motor_a_in2.value(0)
    motor_a_pwm.duty_u16(32768)  # 50% speed
    time.sleep(1)
    
    # Test left motor backward
    print("Left motor backward")
    motor_a_in1.value(0)
    motor_a_in2.value(1)
    motor_a_pwm.duty_u16(32768)  # 50% speed
    time.sleep(1)
    
    # Test right motor forward
    print("Right motor forward")
    motor_b_in1.value(1)
    motor_b_in2.value(0)
    motor_b_pwm.duty_u16(32768)  # 50% speed
    time.sleep(1)
    
    # Test right motor backward
    print("Right motor backward")
    motor_b_in1.value(0)
    motor_b_in2.value(1)
    motor_b_pwm.duty_u16(32768)  # 50% speed
    time.sleep(1)
    
    # Stop all motors
    stop_motors()
    print("Motor test complete")

# Function to test IMU
def test_imu():
    print("Testing IMU...")
    
    for i in range(20):
        angle = read_imu()
        print(f"Current angle: {angle:.2f} degrees")
        time.sleep(0.1)
    
    print("IMU test complete")

# Main function
def main():
    print("Balancing Robot Starting...")
    print("-------------------------")
    
    # Uncomment these lines to test components individually
    # test_motors()
    # test_imu()
    
    # Start the balancing control loop
    balance_robot()

# Run the main function
if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")
        stop_motors()  # Make sure motors are stopped on error