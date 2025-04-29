"""
Raspberry Pi Pico - Intermediate Program 6: Stepper Motor Control
================================================================

This program demonstrates how to control a stepper motor with different
stepping modes (full step, half step, microstepping) and speed control.

Hardware required:
- Raspberry Pi Pico
- 28BYJ-48 stepper motor with ULN2003 driver board (or similar)
- Potentiometer (10K ohm) for speed control
- Pushbutton for direction control
- 10K ohm resistor (for button pull-down)
- Breadboard and jumper wires
- External 5V power supply for the motor (recommended)

Connections:
- Connect ULN2003 IN1 to GPIO pin 2
- Connect ULN2003 IN2 to GPIO pin 3
- Connect ULN2003 IN3 to GPIO pin 4
- Connect ULN2003 IN4 to GPIO pin 5
- Connect ULN2003 VCC to 5V (or external power supply)
- Connect ULN2003 GND to GND
- Connect potentiometer outer pins to 3.3V and GND
- Connect potentiometer middle pin to GPIO pin 26 (ADC0)
- Connect one side of button to 3.3V
- Connect the other side of button to GPIO pin 15 and to GND via 10K resistor

Note: If using an external power supply for the motor, make sure to connect
the ground of the power supply to the Pico's ground.

Instructions:
1. Set up the circuit as described above
2. Upload this program to the Pico
3. Turn the potentiometer to control the motor speed
4. Press the button to change the motor direction

Concepts covered:
- Stepper motor control principles
- Different stepping sequences
- Speed and direction control
- Motor driver interfacing
- Debouncing buttons
"""

import machine
import time
import utime

# Set up the stepper motor pins
in1 = machine.Pin(2, machine.Pin.OUT)
in2 = machine.Pin(3, machine.Pin.OUT)
in3 = machine.Pin(4, machine.Pin.OUT)
in4 = machine.Pin(5, machine.Pin.OUT)

# Set up the potentiometer for speed control
pot = machine.ADC(26)  # ADC0 on GPIO pin 26

# Set up the button for direction control
dir_button = machine.Pin(15, machine.Pin.IN)

# Initialize motor control variables
direction = 1  # 1 for clockwise, -1 for counterclockwise
last_button_time = 0  # For debouncing

# Define stepping sequences

# Full step sequence (highest torque)
FULL_STEP_SEQ = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
]

# Half step sequence (smoother motion)
HALF_STEP_SEQ = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1]
]

# Wave drive sequence (less torque, less power consumption)
WAVE_DRIVE_SEQ = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
]

# Choose which sequence to use
# STEP_SEQ = FULL_STEP_SEQ
STEP_SEQ = HALF_STEP_SEQ
# STEP_SEQ = WAVE_DRIVE_SEQ

# Current position in the sequence
step_counter = 0

def set_motor_pins(step):
    """Set the motor pins according to the current step in the sequence"""
    in1.value(step[0])
    in2.value(step[1])
    in3.value(step[2])
    in4.value(step[3])

def turn_off_motor():
    """Turn off all motor pins to save power and prevent overheating"""
    in1.value(0)
    in2.value(0)
    in3.value(0)
    in4.value(0)

def check_direction_button():
    """Check if the direction button is pressed and change direction"""
    global direction, last_button_time
    
    if dir_button.value() == 1:
        # Debounce
        current_time = utime.ticks_ms()
        if utime.ticks_diff(current_time, last_button_time) > 300:
            # Change direction
            direction *= -1
            print(f"Direction changed: {'Clockwise' if direction > 0 else 'Counterclockwise'}")
            last_button_time = current_time
            time.sleep(0.2)  # Additional debounce delay

def read_speed():
    """Read the potentiometer and convert to a delay value (ms)"""
    pot_value = pot.read_u16()
    # Map 16-bit ADC value (0-65535) to delay (2-20 ms)
    # Lower delay = faster speed
    delay_ms = 2 + (pot_value / 65535) * 18
    return delay_ms

try:
    print("Stepper Motor Control Demo")
    print("=========================")
    print("- Turn potentiometer to control speed")
    print("- Press button to change direction")
    print("Press Ctrl+C to exit")
    print()
    
    while True:
        # Check for direction change
        check_direction_button()
        
        # Read speed from potentiometer
        delay = read_speed()
        
        # Get the current step in the sequence
        step = STEP_SEQ[step_counter]
        
        # Set the motor pins
        set_motor_pins(step)
        
        # Calculate the next step based on direction
        if direction > 0:
            step_counter = (step_counter + 1) % len(STEP_SEQ)
        else:
            step_counter = (step_counter - 1) % len(STEP_SEQ)
        
        # Delay determines the motor speed
        time.sleep(delay / 1000)  # Convert ms to seconds
        
except KeyboardInterrupt:
    # Clean up on exit
    turn_off_motor()
    print("\nProgram stopped")