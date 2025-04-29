"""
Raspberry Pi Pico - Beginner Program 2: Button Input
===================================================

This program demonstrates how to read input from a button
and respond to it by controlling an LED.

Hardware required:
- Raspberry Pi Pico
- Pushbutton
- 10K ohm resistor (for pull-down)
- LED
- 220 ohm resistor (for LED)
- Breadboard and jumper wires

Connections:
- Connect one side of the button to 3.3V
- Connect the other side of the button to GPIO pin 15
- Connect a 10K resistor from GPIO pin 15 to GND (pull-down)
- Connect the LED's anode (longer leg) to GPIO pin 16 through a 220 ohm resistor
- Connect the LED's cathode (shorter leg) to GND

Instructions:
1. Set up the circuit as described above
2. Upload this program to the Pico
3. Press the button to turn on the LED

Concepts covered:
- Digital input with pull-down resistor
- Reading button state
- Conditional statements
- Controlling an output based on input
"""

import machine
import time

# Set up the button on GPIO pin 15 with a pull-down resistor
button = machine.Pin(15, machine.Pin.IN)

# Set up the LED on GPIO pin 16
led = machine.Pin(16, machine.Pin.OUT)

# Main loop
while True:
    # Read the button state
    button_state = button.value()
    
    # If button is pressed (HIGH), turn on the LED
    if button_state == 1:
        led.value(1)  # Turn on LED
    else:
        led.value(0)  # Turn off LED
    
    # Small delay to prevent reading the button too quickly
    time.sleep(0.01)