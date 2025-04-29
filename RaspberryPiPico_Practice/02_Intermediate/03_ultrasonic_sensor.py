"""
Raspberry Pi Pico - Intermediate Program 3: Ultrasonic Distance Sensor
=====================================================================

This program demonstrates how to interface with an HC-SR04 ultrasonic distance
sensor to measure distances.

Hardware required:
- Raspberry Pi Pico
- HC-SR04 ultrasonic distance sensor
- 2 x 10K ohm resistors (for voltage divider)
- Breadboard and jumper wires
- Optional: LED or buzzer for proximity alert

Connections:
- Connect HC-SR04 VCC to 5V (or 3.3V if using a 3.3V compatible sensor)
- Connect HC-SR04 GND to GND
- Connect HC-SR04 TRIG to GPIO pin 14
- Connect HC-SR04 ECHO to a voltage divider (see note below)
- Connect the output of the voltage divider to GPIO pin 15

Note: The HC-SR04 ECHO pin outputs 5V, but Pico GPIO pins are 3.3V tolerant.
Create a voltage divider with two 10K resistors:
- Connect one end of a 10K resistor to the ECHO pin
- Connect the other end to GPIO pin 15 and to one end of another 10K resistor
- Connect the other end of the second 10K resistor to GND

Instructions:
1. Set up the circuit as described above
2. Upload this program to the Pico
3. Open the serial console to view the distance measurements
4. Move objects in front of the sensor to see the distance change

Concepts covered:
- Ultrasonic distance measurement
- Pulse timing and speed of sound calculations
- Voltage dividers for level shifting
- Moving average filtering for stable readings
- Distance-based alerts
"""

import machine
import time
import utime

# Set up the ultrasonic sensor pins
trigger = machine.Pin(14, machine.Pin.OUT)
echo = machine.Pin(15, machine.Pin.IN)

# Optional: Set up an LED for proximity alert
alert_led = machine.Pin(16, machine.Pin.OUT)

# Constants
SOUND_SPEED = 343  # Speed of sound in m/s at 20°C
MAX_DISTANCE = 400  # Maximum distance in cm
TIMEOUT = 30000     # Timeout in microseconds (adjust as needed)

# Initialize variables for moving average filter
readings = [0] * 5  # Store the last 5 readings
reading_index = 0   # Current index in the readings array

def measure_distance():
    """Measure distance using the HC-SR04 ultrasonic sensor"""
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
            return None  # Timeout waiting for echo to start
        pass
    
    # Record the start time of the echo pulse
    pulse_start = utime.ticks_us()
    
    # Wait for echo to go low
    while echo.value() == 1:
        if utime.ticks_diff(utime.ticks_us(), pulse_start) > TIMEOUT:
            return None  # Timeout waiting for echo to end
        pass
    
    # Record the end time of the echo pulse
    pulse_end = utime.ticks_us()
    
    # Calculate the pulse duration in microseconds
    pulse_duration = utime.ticks_diff(pulse_end, pulse_start)
    
    # Calculate the distance in centimeters
    # Distance = (Time × Speed of Sound) ÷ 2
    # The division by 2 is because the sound travels to the object and back
    distance_cm = (pulse_duration * SOUND_SPEED) / 20000
    
    # Check if the distance is within the valid range
    if distance_cm > MAX_DISTANCE:
        return None
    
    return distance_cm

def add_to_moving_average(distance):
    """Add a new reading to the moving average filter"""
    global reading_index
    
    if distance is not None:
        readings[reading_index] = distance
        reading_index = (reading_index + 1) % len(readings)
    
    # Calculate the average of valid readings
    valid_readings = [r for r in readings if r > 0]
    if valid_readings:
        return sum(valid_readings) / len(valid_readings)
    else:
        return None

def set_proximity_alert(distance):
    """Set the alert LED based on proximity"""
    if distance is not None:
        # Turn on the LED if an object is closer than 20 cm
        if distance < 20:
            alert_led.value(1)
        else:
            alert_led.value(0)

try:
    print("HC-SR04 Ultrasonic Distance Sensor Demo")
    print("=======================================")
    print("Press Ctrl+C to exit")
    print()
    
    while True:
        # Measure the distance
        distance = measure_distance()
        
        # Apply moving average filter
        avg_distance = add_to_moving_average(distance)
        
        # Set proximity alert
        set_proximity_alert(avg_distance)
        
        # Display the distance
        if avg_distance is not None:
            print(f"Distance: {avg_distance:.1f} cm")
        else:
            print("Out of range or error in measurement")
        
        # Wait before taking the next measurement
        time.sleep(0.1)
        
except KeyboardInterrupt:
    # Clean up on exit
    alert_led.value(0)
    print("\nProgram stopped")