"""
Raspberry Pi Pico - Beginner Program 7: Temperature Monitor
==========================================================

This program demonstrates how to read the Pico's built-in temperature sensor
and display the readings in the console.

Hardware required:
- Raspberry Pi Pico
- USB cable for power and programming

No additional components are needed as we're using the built-in temperature sensor.

Instructions:
1. Connect your Pico to your computer via USB
2. Upload this program to the Pico
3. Open the serial console to view the temperature readings

Concepts covered:
- Reading the built-in temperature sensor
- Converting ADC readings to temperature
- Periodic measurements
- Formatting output
"""

import machine
import time

# Set up the built-in temperature sensor
# The temperature sensor is connected to ADC4 (GP36)
sensor_temp = machine.ADC(4)

# Conversion factor for the temperature sensor
# The conversion depends on the Pico's operating voltage (3.3V)
conversion_factor = 3.3 / (65535)

def read_temperature():
    # Read the raw ADC value
    raw_value = sensor_temp.read_u16()
    
    # Convert the raw value to voltage
    voltage = raw_value * conversion_factor
    
    # Convert the voltage to temperature in Celsius
    # The formula is from the RP2040 datasheet
    temperature_celsius = 27 - (voltage - 0.706) / 0.001721
    
    return temperature_celsius

try:
    print("Raspberry Pi Pico Temperature Monitor")
    print("------------------------------------")
    print("Press Ctrl+C to exit")
    print()
    
    while True:
        # Read the temperature
        temperature = read_temperature()
        
        # Convert to Fahrenheit
        temperature_fahrenheit = temperature * 9/5 + 32
        
        # Display the temperature
        print(f"Temperature: {temperature:.2f}°C ({temperature_fahrenheit:.2f}°F)")
        
        # Wait for 2 seconds before the next reading
        time.sleep(2)
        
except KeyboardInterrupt:
    print("\nProgram stopped")