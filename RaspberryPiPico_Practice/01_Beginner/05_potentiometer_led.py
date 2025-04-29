"""
Raspberry Pi Pico - Beginner Program 5: Potentiometer-Controlled LED
===================================================================

This program demonstrates how to read analog input from a potentiometer
and use it to control the brightness of an LED using PWM.

Hardware required:
- Raspberry Pi Pico
- Potentiometer (10K ohm recommended)
- LED
- 220 ohm resistor
- Breadboard and jumper wires

Connections:
- Connect the potentiometer's outer pins to 3.3V and GND
- Connect the potentiometer's middle pin to GPIO pin 26 (ADC0)
- Connect the LED's anode (longer leg) to GPIO pin 15 through a 220 ohm resistor
- Connect the LED's cathode (shorter leg) to GND

Instructions:
1. Set up the circuit as described above
2. Upload this program to the Pico
3. Turn the potentiometer to control the LED brightness

Concepts covered:
- Analog-to-Digital Conversion (ADC)
- Reading potentiometer values
- Mapping values from one range to another
- Controlling LED brightness with PWM
"""

import machine
import time

# Set up the ADC (Analog-to-Digital Converter) on GPIO pin 26 (ADC0)
potentiometer = machine.ADC(26)

# Set up the LED pin for PWM
led_pin = machine.Pin(15)
pwm = machine.PWM(led_pin)
pwm.freq(1000)  # Set PWM frequency to 1000 Hz

# Function to map a value from one range to another
def map_value(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

try:
    while True:
        # Read the potentiometer value (0-65535)
        pot_value = potentiometer.read_u16()
        
        # Map the potentiometer value to the PWM range (0-65535)
        # Note: The ADC and PWM on Pico both use 16-bit resolution (0-65535)
        led_brightness = pot_value
        
        # Set the LED brightness
        pwm.duty_u16(led_brightness)
        
        # Print the values for debugging
        print(f"Potentiometer: {pot_value}, LED brightness: {led_brightness}")
        
        # Small delay to prevent flooding the console with messages
        time.sleep(0.1)
        
except KeyboardInterrupt:
    # Clean up on exit
    pwm.duty_u16(0)  # Turn off the LED
    print("Program stopped")