"""
Raspberry Pi Pico - Advanced Program 5: BLE Smart Home Controller
================================================================

This program demonstrates how to create a Bluetooth Low Energy (BLE) smart home
controller that can monitor sensors and control devices from a smartphone app.

Note: This program requires a Raspberry Pi Pico W (with Bluetooth capability).
It will not work on a standard Pico without Bluetooth.

Hardware required:
- Raspberry Pi Pico W
- DHT22 or DHT11 temperature and humidity sensor
- Relay module (1-4 channels)
- PIR motion sensor
- Light-dependent resistor (LDR)
- RGB LED
- 10K ohm resistor (for DHT sensor)
- 10K ohm resistor (for LDR)
- 3 x 220 ohm resistors (for RGB LED)
- Breadboard and jumper wires

Connections:
- DHT sensor:
  - Connect VCC to 3.3V
  - Connect GND to GND
  - Connect DATA to GPIO pin 15
  - Connect a 10K resistor between DATA and VCC (pull-up)

- Relay module:
  - Connect VCC to 5V (or external power if needed)
  - Connect GND to GND
  - Connect IN1 to GPIO pin 16
  - Connect IN2 to GPIO pin 17 (if using multiple relays)

- PIR motion sensor:
  - Connect VCC to 3.3V
  - Connect GND to GND
  - Connect OUT to GPIO pin 14

- Light-dependent resistor (LDR):
  - Connect one leg of LDR to 3.3V
  - Connect other leg to GPIO pin 26 (ADC0) and to GND via 10K resistor

- RGB LED:
  - Connect Red pin to GPIO pin 2 through a 220 ohm resistor
  - Connect Green pin to GPIO pin 3 through a 220 ohm resistor
  - Connect Blue pin to GPIO pin 4 through a 220 ohm resistor
  - Connect common cathode to GND (or common anode to 3.3V and adjust code)

Note: You'll need to install the following libraries:
- dht.py for the DHT sensor
- bluetooth.py and aioble.py for BLE functionality

Instructions:
1. Set up the circuit as described above
2. Upload the required libraries to your Pico W
3. Upload this program to the Pico W
4. Use a BLE scanner app on your smartphone to find and connect to the device
5. Control your smart home devices and monitor sensors through the app

Concepts covered:
- Bluetooth Low Energy (BLE) communication
- GATT services and characteristics
- Asynchronous programming with asyncio
- Sensor data collection and processing
- Remote device control
- Smart home automation concepts
"""

import machine
import time
import struct
import json
import asyncio
import gc

# Import required libraries
try:
    import dht
    import bluetooth
    import aioble
    from micropython import const
except ImportError:
    print("Error: Required libraries not found")
    print("Please upload dht.py, bluetooth.py, and aioble.py to your Pico")
    raise

# BLE UUIDs
DEVICE_INFO_SERVICE_UUID = bluetooth.UUID(0x180A)
MANUFACTURER_CHAR_UUID = bluetooth.UUID(0x2A29)

ENVIRONMENTAL_SERVICE_UUID = bluetooth.UUID('19B10000-E8F2-537E-4F6C-D104768A1214')
TEMPERATURE_CHAR_UUID = bluetooth.UUID('19B10001-E8F2-537E-4F6C-D104768A1214')
HUMIDITY_CHAR_UUID = bluetooth.UUID('19B10002-E8F2-537E-4F6C-D104768A1214')
LIGHT_CHAR_UUID = bluetooth.UUID('19B10003-E8F2-537E-4F6C-D104768A1214')
MOTION_CHAR_UUID = bluetooth.UUID('19B10004-E8F2-537E-4F6C-D104768A1214')

CONTROL_SERVICE_UUID = bluetooth.UUID('19B20000-E8F2-537E-4F6C-D104768A1214')
RELAY1_CHAR_UUID = bluetooth.UUID('19B20001-E8F2-537E-4F6C-D104768A1214')
RELAY2_CHAR_UUID = bluetooth.UUID('19B20002-E8F2-537E-4F6C-D104768A1214')
RGB_LED_CHAR_UUID = bluetooth.UUID('19B20003-E8F2-537E-4F6C-D104768A1214')

# Set up the sensors and actuators
# DHT sensor
dht_pin = machine.Pin(15)
dht_sensor = dht.DHT22(dht_pin)  # Change to DHT11 if using that sensor

# Relay module
relay1 = machine.Pin(16, machine.Pin.OUT)
relay2 = machine.Pin(17, machine.Pin.OUT)

# PIR motion sensor
pir_sensor = machine.Pin(14, machine.Pin.IN)

# Light-dependent resistor (LDR)
ldr = machine.ADC(26)  # ADC0 on GPIO pin 26

# RGB LED
red_led = machine.Pin(2, machine.Pin.OUT)
green_led = machine.Pin(3, machine.Pin.OUT)
blue_led = machine.Pin(4, machine.Pin.OUT)

# Global variables for sensor data
temperature = 0.0
humidity = 0.0
light_level = 0
motion_detected = False

# Global variables for control states
relay1_state = False
relay2_state = False
rgb_color = (0, 0, 0)  # (R, G, B)

# Function to read DHT sensor
def read_dht():
    global temperature, humidity
    try:
        dht_sensor.measure()
        temperature = dht_sensor.temperature()
        humidity = dht_sensor.humidity()
        print(f"Temperature: {temperature:.1f}°C, Humidity: {humidity:.1f}%")
    except Exception as e:
        print(f"Error reading DHT sensor: {e}")

# Function to read light level from LDR
def read_light():
    global light_level
    try:
        # Read the raw ADC value (0-65535)
        raw_value = ldr.read_u16()
        
        # Convert to percentage (0-100)
        # Invert the value since higher resistance = lower light
        light_level = 100 - int((raw_value / 65535) * 100)
        print(f"Light level: {light_level}%")
    except Exception as e:
        print(f"Error reading light sensor: {e}")

# Function to read motion sensor
def read_motion():
    global motion_detected
    try:
        motion_detected = bool(pir_sensor.value())
        print(f"Motion detected: {motion_detected}")
    except Exception as e:
        print(f"Error reading motion sensor: {e}")

# Function to control relay 1
def set_relay1(state):
    global relay1_state
    try:
        relay1.value(1 if state else 0)
        relay1_state = state
        print(f"Relay 1 set to: {'ON' if state else 'OFF'}")
    except Exception as e:
        print(f"Error controlling relay 1: {e}")

# Function to control relay 2
def set_relay2(state):
    global relay2_state
    try:
        relay2.value(1 if state else 0)
        relay2_state = state
        print(f"Relay 2 set to: {'ON' if state else 'OFF'}")
    except Exception as e:
        print(f"Error controlling relay 2: {e}")

# Function to control RGB LED
def set_rgb_led(r, g, b):
    global rgb_color
    try:
        red_led.value(1 if r > 0 else 0)
        green_led.value(1 if g > 0 else 0)
        blue_led.value(1 if b > 0 else 0)
        rgb_color = (r, g, b)
        print(f"RGB LED set to: ({r}, {g}, {b})")
    except Exception as e:
        print(f"Error controlling RGB LED: {e}")

# Function to read all sensors
async def read_all_sensors():
    while True:
        read_dht()
        read_light()
        read_motion()
        await asyncio.sleep(2)  # Read sensors every 2 seconds

# Set up BLE services and characteristics
# Device Information Service
device_info_service = aioble.Service(DEVICE_INFO_SERVICE_UUID)
manufacturer_char = aioble.Characteristic(
    device_info_service,
    MANUFACTURER_CHAR_UUID,
    read=True,
    initial="Raspberry Pi Pico W Smart Home"
)

# Environmental Monitoring Service
environmental_service = aioble.Service(ENVIRONMENTAL_SERVICE_UUID)

temperature_char = aioble.Characteristic(
    environmental_service,
    TEMPERATURE_CHAR_UUID,
    read=True, notify=True,
    initial=struct.pack("f", temperature)
)

humidity_char = aioble.Characteristic(
    environmental_service,
    HUMIDITY_CHAR_UUID,
    read=True, notify=True,
    initial=struct.pack("f", humidity)
)

light_char = aioble.Characteristic(
    environmental_service,
    LIGHT_CHAR_UUID,
    read=True, notify=True,
    initial=struct.pack("i", light_level)
)

motion_char = aioble.Characteristic(
    environmental_service,
    MOTION_CHAR_UUID,
    read=True, notify=True,
    initial=struct.pack("?", motion_detected)
)

# Control Service
control_service = aioble.Service(CONTROL_SERVICE_UUID)

relay1_char = aioble.Characteristic(
    control_service,
    RELAY1_CHAR_UUID,
    read=True, write=True,
    initial=struct.pack("?", relay1_state)
)

relay2_char = aioble.Characteristic(
    control_service,
    RELAY2_CHAR_UUID,
    read=True, write=True,
    initial=struct.pack("?", relay2_state)
)

rgb_led_char = aioble.Characteristic(
    control_service,
    RGB_LED_CHAR_UUID,
    read=True, write=True,
    initial=struct.pack("BBB", *rgb_color)
)

# Register all services
aioble.register_services(device_info_service, environmental_service, control_service)

# Function to update characteristic values
async def update_characteristics():
    while True:
        # Update environmental characteristics
        temperature_char.write(struct.pack("f", temperature))
        humidity_char.write(struct.pack("f", humidity))
        light_char.write(struct.pack("i", light_level))
        motion_char.write(struct.pack("?", motion_detected))
        
        # Notify connected clients
        await temperature_char.notify()
        await humidity_char.notify()
        await light_char.notify()
        await motion_char.notify()
        
        await asyncio.sleep(1)  # Update every second

# Function to handle relay1 characteristic writes
async def handle_relay1_writes():
    while True:
        # Wait for a write to the characteristic
        await relay1_char.written()
        
        # Get the new value
        value = struct.unpack("?", relay1_char.read())[0]
        
        # Set the relay
        set_relay1(value)

# Function to handle relay2 characteristic writes
async def handle_relay2_writes():
    while True:
        # Wait for a write to the characteristic
        await relay2_char.written()
        
        # Get the new value
        value = struct.unpack("?", relay2_char.read())[0]
        
        # Set the relay
        set_relay2(value)

# Function to handle RGB LED characteristic writes
async def handle_rgb_led_writes():
    while True:
        # Wait for a write to the characteristic
        await rgb_led_char.written()
        
        # Get the new value
        r, g, b = struct.unpack("BBB", rgb_led_char.read())
        
        # Set the RGB LED
        set_rgb_led(r, g, b)

# Function to handle BLE advertising
async def advertise():
    while True:
        # Advertise the device
        print("Starting BLE advertisement...")
        
        # Create advertisement data
        adv_data = bytearray()
        adv_data.extend(struct.pack("BB", 0x02, 0x01))  # Flags
        adv_data.extend(struct.pack("B", 0x06))         # General Discoverable
        
        # Add device name
        device_name = "Pico Smart Home"
        adv_data.extend(struct.pack("BB", len(device_name) + 1, 0x09))
        adv_data.extend(device_name.encode())
        
        # Add service UUIDs
        adv_data.extend(struct.pack("B", 0x03))  # Length
        adv_data.extend(struct.pack("B", 0x03))  # Complete list of 16-bit UUIDs
        adv_data.extend(struct.pack("<H", 0x180A))  # Device Information Service
        
        # Start advertising
        connection = await aioble.advertise(
            adv_data,
            interval_us=100000,  # 100ms interval
            timeout_ms=0  # No timeout
        )
        
        print(f"Connection from {connection.device}")
        
        # Wait for disconnection
        await connection.disconnected()
        
        print("Device disconnected")
        
        # Free up memory
        gc.collect()

# Main function
async def main():
    # Initialize devices
    set_relay1(False)
    set_relay2(False)
    set_rgb_led(0, 0, 0)
    
    print("Smart Home Controller Starting...")
    print("--------------------------------")
    
    # Create tasks for all the async functions
    sensor_task = asyncio.create_task(read_all_sensors())
    update_task = asyncio.create_task(update_characteristics())
    relay1_task = asyncio.create_task(handle_relay1_writes())
    relay2_task = asyncio.create_task(handle_relay2_writes())
    rgb_task = asyncio.create_task(handle_rgb_led_writes())
    advertise_task = asyncio.create_task(advertise())
    
    # Wait for all tasks to complete (they won't unless there's an error)
    await asyncio.gather(
        sensor_task,
        update_task,
        relay1_task,
        relay2_task,
        rgb_task,
        advertise_task
    )

# Run the main function
try:
    asyncio.run(main())
except KeyboardInterrupt:
    # Clean up
    set_relay1(False)
    set_relay2(False)
    set_rgb_led(0, 0, 0)
    print("\nSmart Home Controller stopped by user")
except Exception as e:
    print(f"Error: {e}")
    # Clean up
    set_relay1(False)
    set_relay2(False)
    set_rgb_led(0, 0, 0)