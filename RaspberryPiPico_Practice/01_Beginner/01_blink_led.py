"""
Raspberry Pi Pico - Beginner Program 1: Blinking LED
====================================================

This program demonstrates the most basic operation with Raspberry Pi Pico:
blinking the onboard LED.

Hardware required:
- Raspberry Pi Pico
- USB cable for power and programming

The Pico has an onboard LED connected to GPIO pin 25.
This program will make it blink on and off at regular intervals.

Instructions:
1. Connect your Pico to your computer via USB
2. Upload this program to the Pico
3. The onboard LED should start blinking

Concepts covered:
- Basic MicroPython imports
- GPIO pin configuration
- Simple timing with sleep()
- Infinite loops
"""

import machine  # Core functionality for hardware control
import time     # For timing and delays

# Set up the onboard LED (connected to GPIO pin 25 on the Pico)
led = machine.Pin(25, machine.Pin.OUT)

# Main loop that runs forever
while True:
    led.value(1)  # Turn on the LED
    time.sleep(0.5)  # Wait for 0.5 seconds
    
    led.value(0)  # Turn off the LED
    time.sleep(0.5)  # Wait for 0.5 seconds
    
    # The loop will repeat, creating a blinking effect