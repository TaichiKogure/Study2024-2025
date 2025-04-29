"""
Raspberry Pi Pico - Intermediate Program 2: DHT Temperature & Humidity Sensor
============================================================================

This program demonstrates how to interface with a DHT22 or DHT11 temperature
and humidity sensor and display the readings.

Hardware required:
- Raspberry Pi Pico
- DHT22 or DHT11 temperature and humidity sensor
- 10K ohm resistor (pull-up)
- Breadboard and jumper wires
- Optional: OLED display (if you want to display readings on a screen)

Connections:
- Connect DHT VCC to 3.3V
- Connect DHT GND to GND
- Connect DHT DATA to GPIO pin 15
- Connect a 10K resistor between DATA and VCC (pull-up)

Note: You'll need to install the DHT library. Upload the dht.py
file to your Pico before running this program.

Instructions:
1. Set up the circuit as described above
2. Upload the dht.py library to your Pico
3. Upload this program to the Pico
4. Open the serial console to view the temperature and humidity readings

Concepts covered:
- Working with environmental sensors
- Custom one-wire communication protocol
- Reading multiple sensor values
- Error handling for sensor readings
- Periodic measurements and logging
"""

import machine
import time
import utime

# Import the DHT library
# Note: You need to upload dht.py to your Pico first
try:
    import dht
except ImportError:
    print("Error: dht library not found")
    print("Please upload dht.py to your Pico")
    raise

# Set up the DHT sensor
# Use dht.DHT22 for DHT22 sensor or dht.DHT11 for DHT11 sensor
sensor_pin = machine.Pin(15)
sensor = dht.DHT22(sensor_pin)  # Change to DHT11 if you're using that sensor

# Function to read sensor data with error handling
def read_sensor():
    try:
        sensor.measure()  # Trigger a measurement
        temperature = sensor.temperature()  # Get temperature in Celsius
        humidity = sensor.humidity()  # Get humidity percentage
        return temperature, humidity
    except Exception as e:
        print(f"Error reading sensor: {e}")
        return None, None

# Function to convert Celsius to Fahrenheit
def celsius_to_fahrenheit(celsius):
    return celsius * 9/5 + 32

# Function to calculate heat index (apparent temperature)
def calculate_heat_index(temperature, humidity):
    # This is a simplified version of the heat index formula
    t = celsius_to_fahrenheit(temperature)
    h = humidity
    
    # Basic heat index calculation
    hi = 0.5 * (t + 61.0 + ((t - 68.0) * 1.2) + (h * 0.094))
    
    # For more extreme conditions, use the full formula
    if hi > 80:
        hi = -42.379 + 2.04901523 * t + 10.14333127 * h
        hi = hi - 0.22475541 * t * h - 0.00683783 * t * t
        hi = hi - 0.05481717 * h * h + 0.00122874 * t * t * h
        hi = hi + 0.00085282 * t * h * h - 0.00000199 * t * t * h * h
    
    # Convert back to Celsius
    return (hi - 32) * 5/9

try:
    print("DHT Temperature & Humidity Sensor Demo")
    print("======================================")
    print("Press Ctrl+C to exit")
    print()
    
    # Wait for the sensor to stabilize
    time.sleep(2)
    
    while True:
        # Read sensor data
        temperature, humidity = read_sensor()
        
        if temperature is not None and humidity is not None:
            # Calculate additional values
            fahrenheit = celsius_to_fahrenheit(temperature)
            heat_index = calculate_heat_index(temperature, humidity)
            
            # Display the readings
            print(f"Temperature: {temperature:.1f}°C ({fahrenheit:.1f}°F)")
            print(f"Humidity: {humidity:.1f}%")
            print(f"Heat Index: {heat_index:.1f}°C")
            print("-" * 30)
        else:
            print("Failed to read sensor data. Retrying...")
        
        # Wait before taking the next reading
        # DHT sensors typically need at least 2 seconds between readings
        time.sleep(3)
        
except KeyboardInterrupt:
    print("\nProgram stopped")