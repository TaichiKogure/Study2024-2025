"""
Raspberry Pi Pico - Beginner Program 3: PWM LED Fading
=====================================================

This program demonstrates how to use Pulse Width Modulation (PWM)
to fade an LED in and out, creating a breathing effect.

Hardware required:
- Raspberry Pi Pico
- LED
- 220 ohm resistor
- Breadboard and jumper wires

Connections:
- Connect the LED's anode (longer leg) to GPIO pin 15 through a 220 ohm resistor
- Connect the LED's cathode (shorter leg) to GND

Instructions:
1. Set up the circuit as described above
2. Upload this program to the Pico
3. The LED should start fading in and out

Concepts covered:
- Pulse Width Modulation (PWM)
- Analog-like output from digital pins
- Using loops to create patterns
- Working with duty cycles
"""

import machine
import time

# Set up the LED pin
led_pin = machine.Pin(15)

# Create a PWM object on the LED pin
# Frequency of 1000 Hz is a good starting point for LEDs
pwm = machine.PWM(led_pin)
pwm.freq(1000)

# Set the initial duty cycle to 0 (LED off)
# PWM duty cycle on Pico ranges from 0 (off) to 65535 (fully on)
pwm.duty_u16(0)

try:
    while True:
        # Fade in: gradually increase brightness
        for duty in range(0, 65536, 1024):  # Step by ~1.5% each time
            pwm.duty_u16(duty)
            time.sleep(0.01)
        
        # Hold at maximum brightness for a moment
        time.sleep(0.2)
        
        # Fade out: gradually decrease brightness
        for duty in range(65535, -1, -1024):  # Step by ~1.5% each time
            pwm.duty_u16(duty)
            time.sleep(0.01)
        
        # Hold at minimum brightness for a moment
        time.sleep(0.2)

except KeyboardInterrupt:
    # Clean up on exit
    pwm.duty_u16(0)  # Turn off the LED