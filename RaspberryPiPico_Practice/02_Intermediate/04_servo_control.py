"""
Raspberry Pi Pico - Intermediate Program 4: Servo Motor Control
==============================================================

This program demonstrates how to control servo motors using PWM signals,
with different control modes including potentiometer, button, and automatic sweeping.

Hardware required:
- Raspberry Pi Pico
- Servo motor (standard hobby servo)
- Potentiometer (10K ohm)
- 2 pushbuttons
- 10K ohm resistors (for button pull-downs)
- Breadboard and jumper wires
- External power supply for servo (recommended for larger servos)

Connections:
- Connect servo signal wire to GPIO pin 15
- Connect servo power to 5V (or external power supply for larger servos)
- Connect servo ground to GND
- Connect potentiometer outer pins to 3.3V and GND
- Connect potentiometer middle pin to GPIO pin 26 (ADC0)
- Connect one side of button 1 to 3.3V
- Connect the other side of button 1 to GPIO pin 14 and to GND via 10K resistor
- Connect one side of button 2 to 3.3V
- Connect the other side of button 2 to GPIO pin 13 and to GND via 10K resistor

Note: If using an external power supply for the servo, make sure to connect
the ground of the power supply to the Pico's ground.

Instructions:
1. Set up the circuit as described above
2. Upload this program to the Pico
3. Use the buttons to switch between control modes:
   - Button 1: Switch to the next mode
   - Button 2: In button control mode, press to move the servo
4. In potentiometer mode, turn the potentiometer to control the servo position

Concepts covered:
- Servo motor control with PWM
- Multiple input methods (potentiometer, buttons)
- State machine for mode switching
- Debouncing buttons
- Interpolation for smooth movement
"""

import machine
import time
import utime

# Set up the servo pin for PWM
servo_pin = machine.Pin(15)
servo_pwm = machine.PWM(servo_pin)
servo_pwm.freq(50)  # Standard servo frequency is 50Hz

# Set up the potentiometer
pot = machine.ADC(26)  # ADC0 on GPIO pin 26

# Set up the buttons with pull-down resistors
mode_button = machine.Pin(14, machine.Pin.IN)
control_button = machine.Pin(13, machine.Pin.IN)

# Constants for servo control
MIN_DUTY = 1000  # Minimum duty cycle (0 degrees)
MAX_DUTY = 9000  # Maximum duty cycle (180 degrees)
MID_DUTY = (MIN_DUTY + MAX_DUTY) // 2  # Middle position (90 degrees)

# Control modes
MODE_POT = 0      # Control with potentiometer
MODE_BUTTON = 1   # Control with button (increment position)
MODE_SWEEP = 2    # Automatic sweeping
MODE_NAMES = ["Potentiometer", "Button", "Auto Sweep"]
current_mode = MODE_POT

# Variables for button control
button_pos = 90  # Current position in degrees (0-180)
last_button_time = 0  # For debouncing

# Variables for sweep mode
sweep_pos = 0  # Current position in sweep (0-180)
sweep_dir = 1  # Direction of sweep (1 = increasing, -1 = decreasing)
last_sweep_time = 0  # For controlling sweep speed

# Variables for mode switching
last_mode_button_time = 0  # For debouncing

def set_servo_position(degrees):
    """Set the servo position in degrees (0-180)"""
    # Convert degrees to duty cycle
    duty = MIN_DUTY + (degrees / 180) * (MAX_DUTY - MIN_DUTY)
    servo_pwm.duty_u16(int(duty))

def read_pot_position():
    """Read the potentiometer and convert to degrees (0-180)"""
    pot_value = pot.read_u16()
    # Map 16-bit ADC value (0-65535) to degrees (0-180)
    degrees = (pot_value / 65535) * 180
    return degrees

def check_mode_button():
    """Check if the mode button is pressed and switch modes"""
    global current_mode, last_mode_button_time
    
    if mode_button.value() == 1:
        # Debounce
        current_time = utime.ticks_ms()
        if utime.ticks_diff(current_time, last_mode_button_time) > 300:
            # Switch to next mode
            current_mode = (current_mode + 1) % len(MODE_NAMES)
            print(f"Switched to mode: {MODE_NAMES[current_mode]}")
            last_mode_button_time = current_time
            time.sleep(0.2)  # Additional debounce delay

def check_control_button():
    """Check if the control button is pressed and update position"""
    global button_pos, last_button_time
    
    if control_button.value() == 1:
        # Debounce
        current_time = utime.ticks_ms()
        if utime.ticks_diff(current_time, last_button_time) > 200:
            # Increment position by 15 degrees
            button_pos = (button_pos + 15) % 181
            print(f"Button position: {button_pos} degrees")
            set_servo_position(button_pos)
            last_button_time = current_time

def update_sweep():
    """Update the automatic sweep position"""
    global sweep_pos, sweep_dir, last_sweep_time
    
    current_time = utime.ticks_ms()
    if utime.ticks_diff(current_time, last_sweep_time) > 20:  # Control sweep speed
        # Update position
        sweep_pos += sweep_dir
        
        # Change direction at limits
        if sweep_pos >= 180:
            sweep_pos = 180
            sweep_dir = -1
        elif sweep_pos <= 0:
            sweep_pos = 0
            sweep_dir = 1
        
        set_servo_position(sweep_pos)
        last_sweep_time = current_time

try:
    print("Servo Motor Control Demo")
    print("=======================")
    print("Press Ctrl+C to exit")
    print(f"Starting in mode: {MODE_NAMES[current_mode]}")
    print()
    
    # Center the servo initially
    set_servo_position(90)
    time.sleep(1)
    
    while True:
        # Check for mode change
        check_mode_button()
        
        # Handle current mode
        if current_mode == MODE_POT:
            # Potentiometer control
            position = read_pot_position()
            set_servo_position(position)
            
        elif current_mode == MODE_BUTTON:
            # Button control
            check_control_button()
            
        elif current_mode == MODE_SWEEP:
            # Automatic sweep
            update_sweep()
        
        # Small delay to prevent CPU hogging
        time.sleep(0.01)
        
except KeyboardInterrupt:
    # Clean up on exit
    set_servo_position(90)  # Return to center position
    time.sleep(0.5)
    servo_pwm.duty_u16(0)  # Turn off the servo
    print("\nProgram stopped")