"""
Raspberry Pi Pico - Advanced Program 3: Multitasking Robot Control
==================================================================

This program demonstrates how to use cooperative multitasking to control
multiple components of a robot simultaneously using the _thread module.

Hardware required:
- Raspberry Pi Pico
- L298N motor driver (or similar)
- 2 DC motors
- Ultrasonic sensor (HC-SR04)
- Servo motor (for sensor or gripper)
- 2 x 10K ohm resistors (for voltage divider)
- Breadboard and jumper wires
- Battery pack (for motors)

Connections:
- Motor Driver:
  - Connect IN1 to GPIO pin 2
  - Connect IN2 to GPIO pin 3
  - Connect IN3 to GPIO pin 4
  - Connect IN4 to GPIO pin 5
  - Connect ENA to GPIO pin 6 (PWM)
  - Connect ENB to GPIO pin 7 (PWM)
  - Connect motor power to external battery
  - Connect GND to common ground

- Ultrasonic Sensor:
  - Connect VCC to 5V (or 3.3V if using a 3.3V compatible sensor)
  - Connect GND to GND
  - Connect TRIG to GPIO pin 14
  - Connect ECHO to a voltage divider (see note below)
  - Connect the output of the voltage divider to GPIO pin 15

- Servo Motor:
  - Connect signal wire to GPIO pin 16
  - Connect power to 5V (or external power supply)
  - Connect ground to GND

Note: The HC-SR04 ECHO pin outputs 5V, but Pico GPIO pins are 3.3V tolerant.
Create a voltage divider with two 10K resistors:
- Connect one end of a 10K resistor to the ECHO pin
- Connect the other end to GPIO pin 15 and to one end of another 10K resistor
- Connect the other end of the second 10K resistor to GND

Instructions:
1. Set up the circuit as described above
2. Upload this program to the Pico
3. The robot will move forward until it detects an obstacle
4. When an obstacle is detected, it will stop, scan with the servo, and turn
5. The program uses multiple threads to handle different tasks simultaneously

Concepts covered:
- Cooperative multitasking with _thread
- Thread synchronization with locks
- Motor control with PWM
- Distance sensing and obstacle avoidance
- Servo control for sensor movement
- State machine for robot behavior
"""

import machine
import time
import utime
import _thread
import random

# Set up the motor driver pins
motor_a_in1 = machine.Pin(2, machine.Pin.OUT)
motor_a_in2 = machine.Pin(3, machine.Pin.OUT)
motor_b_in1 = machine.Pin(4, machine.Pin.OUT)
motor_b_in2 = machine.Pin(5, machine.Pin.OUT)

# Set up PWM for motor speed control
motor_a_enable = machine.PWM(machine.Pin(6))
motor_b_enable = machine.PWM(machine.Pin(7))
motor_a_enable.freq(1000)
motor_b_enable.freq(1000)

# Set up the ultrasonic sensor pins
trigger = machine.Pin(14, machine.Pin.OUT)
echo = machine.Pin(15, machine.Pin.IN)

# Set up the servo pin for PWM
servo_pin = machine.Pin(16)
servo_pwm = machine.PWM(servo_pin)
servo_pwm.freq(50)  # Standard servo frequency is 50Hz

# Constants
SOUND_SPEED = 343  # Speed of sound in m/s at 20°C
MAX_DISTANCE = 200  # Maximum distance in cm
TIMEOUT = 30000     # Timeout in microseconds
MIN_OBSTACLE_DISTANCE = 20  # Minimum distance to obstacle in cm

# Thread synchronization
distance_lock = _thread.allocate_lock()
motor_lock = _thread.allocate_lock()
servo_lock = _thread.allocate_lock()

# Global variables
current_distance = 100  # Initial distance value (cm)
robot_state = "FORWARD"  # Initial robot state
servo_position = 90      # Initial servo position (center)

# Function to set servo position (0-180 degrees)
def set_servo_position(degrees):
    global servo_position
    
    with servo_lock:
        # Clamp the degrees to 0-180
        degrees = max(0, min(180, degrees))
        servo_position = degrees
        
        # Convert degrees to duty cycle
        min_duty = 1000  # 1ms pulse (0 degrees)
        max_duty = 9000  # 2ms pulse (180 degrees)
        duty = min_duty + (degrees / 180) * (max_duty - min_duty)
        servo_pwm.duty_u16(int(duty))

# Function to measure distance using the ultrasonic sensor
def measure_distance():
    # Clear the trigger
    trigger.value(0)
    utime.sleep_us(5)
    
    # Send a 10us pulse to trigger
    trigger.value(1)
    utime.sleep_us(10)
    trigger.value(0)
    
    # Wait for the echo pulse
    pulse_start = utime.ticks_us()
    pulse_timeout = utime.ticks_add(pulse_start, TIMEOUT)
    
    # Wait for echo to go high
    while echo.value() == 0:
        if utime.ticks_diff(utime.ticks_us(), pulse_start) > TIMEOUT:
            return MAX_DISTANCE  # Timeout waiting for echo to start
        pass
    
    # Record the start time of the echo pulse
    pulse_start = utime.ticks_us()
    
    # Wait for echo to go low
    while echo.value() == 1:
        if utime.ticks_diff(utime.ticks_us(), pulse_start) > TIMEOUT:
            return MAX_DISTANCE  # Timeout waiting for echo to end
        pass
    
    # Record the end time of the echo pulse
    pulse_end = utime.ticks_us()
    
    # Calculate the pulse duration in microseconds
    pulse_duration = utime.ticks_diff(pulse_end, pulse_start)
    
    # Calculate the distance in centimeters
    # Distance = (Time × Speed of Sound) ÷ 2
    distance_cm = (pulse_duration * SOUND_SPEED) / 20000
    
    # Check if the distance is within the valid range
    if distance_cm > MAX_DISTANCE:
        return MAX_DISTANCE
    
    return distance_cm

# Function to control the motors
def set_motors(left_speed, right_speed):
    with motor_lock:
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
        
        # Set motor speeds (0-100 to 0-65535)
        left_duty = int((left_speed / 100) * 65535)
        right_duty = int((right_speed / 100) * 65535)
        
        motor_a_enable.duty_u16(left_duty)
        motor_b_enable.duty_u16(right_duty)

# Function to stop all motors
def stop_motors():
    with motor_lock:
        motor_a_in1.value(0)
        motor_a_in2.value(0)
        motor_b_in1.value(0)
        motor_b_in2.value(0)
        motor_a_enable.duty_u16(0)
        motor_b_enable.duty_u16(0)

# Function to scan the environment with the servo
def scan_environment():
    # Scan from left to right
    distances = []
    
    for angle in range(0, 181, 20):  # Scan in 20-degree increments
        set_servo_position(angle)
        time.sleep(0.2)  # Wait for servo to reach position
        
        # Measure distance at this angle
        dist = measure_distance()
        distances.append((angle, dist))
        print(f"Angle: {angle}°, Distance: {dist} cm")
    
    # Find the angle with the maximum distance
    max_distance_angle = max(distances, key=lambda x: x[1])
    print(f"Best direction: {max_distance_angle[0]}°, Distance: {max_distance_angle[1]} cm")
    
    # Return to center position
    set_servo_position(90)
    time.sleep(0.2)
    
    return max_distance_angle[0]  # Return the best angle

# Thread function for distance sensing
def distance_sensing_thread():
    global current_distance
    
    while True:
        # Measure the distance
        distance = measure_distance()
        
        # Update the global distance with thread safety
        with distance_lock:
            current_distance = distance
        
        # Small delay between measurements
        time.sleep(0.1)

# Thread function for servo control
def servo_control_thread():
    global servo_position, robot_state
    
    while True:
        # Check if we need to scan (when robot is in SCANNING state)
        if robot_state == "SCANNING":
            print("Scanning environment...")
            best_angle = scan_environment()
            
            # Determine turn direction based on scan results
            with motor_lock:
                if best_angle < 80:
                    robot_state = "TURNING_LEFT"
                    print("Turning left...")
                elif best_angle > 100:
                    robot_state = "TURNING_RIGHT"
                    print("Turning right...")
                else:
                    # If best direction is forward but distance is still too close,
                    # choose a random direction
                    if current_distance < MIN_OBSTACLE_DISTANCE:
                        if random.choice([True, False]):
                            robot_state = "TURNING_LEFT"
                            print("Randomly turning left...")
                        else:
                            robot_state = "TURNING_RIGHT"
                            print("Randomly turning right...")
                    else:
                        robot_state = "FORWARD"
                        print("Moving forward...")
        
        # Small delay to prevent CPU hogging
        time.sleep(0.1)

# Main robot control function
def robot_control():
    global robot_state
    
    try:
        print("Robot control started")
        print("Press Ctrl+C to stop")
        
        while True:
            # Get the current distance with thread safety
            with distance_lock:
                distance = current_distance
            
            # State machine for robot behavior
            if robot_state == "FORWARD":
                if distance < MIN_OBSTACLE_DISTANCE:
                    # Obstacle detected, stop and scan
                    stop_motors()
                    robot_state = "SCANNING"
                    print(f"Obstacle detected at {distance} cm")
                else:
                    # No obstacle, move forward
                    set_motors(70, 70)  # Both motors forward at 70% speed
            
            elif robot_state == "TURNING_LEFT":
                # Turn left for a short time
                set_motors(-50, 50)  # Left motor backward, right motor forward
                time.sleep(0.5)  # Turn for 0.5 seconds
                
                # After turning, go back to forward state
                robot_state = "FORWARD"
            
            elif robot_state == "TURNING_RIGHT":
                # Turn right for a short time
                set_motors(50, -50)  # Left motor forward, right motor backward
                time.sleep(0.5)  # Turn for 0.5 seconds
                
                # After turning, go back to forward state
                robot_state = "FORWARD"
            
            # Small delay to prevent CPU hogging
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nRobot control stopped by user")
    finally:
        # Clean up
        stop_motors()
        set_servo_position(90)  # Center the servo
        print("Robot shutdown complete")

# Main function
def main():
    # Initialize the servo to center position
    set_servo_position(90)
    
    # Start the distance sensing thread
    _thread.start_new_thread(distance_sensing_thread, ())
    
    # Start the servo control thread
    _thread.start_new_thread(servo_control_thread, ())
    
    # Run the main robot control function
    robot_control()

# Run the main function
if __name__ == "__main__":
    main()