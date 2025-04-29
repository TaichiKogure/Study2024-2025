"""
Raspberry Pi Pico - Beginner Program 4: Traffic Light Simulation
===============================================================

This program simulates a traffic light sequence using three LEDs.

Hardware required:
- Raspberry Pi Pico
- 3 LEDs (Red, Yellow, Green)
- 3 x 220 ohm resistors
- Breadboard and jumper wires

Connections:
- Connect the Red LED to GPIO pin 15 through a 220 ohm resistor
- Connect the Yellow LED to GPIO pin 14 through a 220 ohm resistor
- Connect the Green LED to GPIO pin 13 through a 220 ohm resistor
- Connect all LED cathodes (shorter legs) to GND

Instructions:
1. Set up the circuit as described above
2. Upload this program to the Pico
3. Watch the traffic light sequence

Concepts covered:
- Controlling multiple GPIO pins
- Creating timed sequences
- Using functions to organize code
- Basic state machine concepts
"""

import machine
import time

# Set up the LED pins
red_led = machine.Pin(15, machine.Pin.OUT)
yellow_led = machine.Pin(14, machine.Pin.OUT)
green_led = machine.Pin(13, machine.Pin.OUT)

# Function to turn all LEDs off
def all_leds_off():
    red_led.value(0)
    yellow_led.value(0)
    green_led.value(0)

# Traffic light sequence function
def traffic_light_sequence():
    # Red light
    all_leds_off()
    red_led.value(1)
    print("Red light - Stop")
    time.sleep(3)
    
    # Red and Yellow light (UK/EU sequence)
    yellow_led.value(1)
    print("Red and Yellow light - Get ready")
    time.sleep(1)
    
    # Green light
    all_leds_off()
    green_led.value(1)
    print("Green light - Go")
    time.sleep(3)
    
    # Yellow light
    all_leds_off()
    yellow_led.value(1)
    print("Yellow light - Prepare to stop")
    time.sleep(2)

# Main loop
try:
    print("Traffic light simulation started")
    while True:
        traffic_light_sequence()
        
except KeyboardInterrupt:
    # Clean up on exit
    all_leds_off()
    print("Traffic light simulation stopped")