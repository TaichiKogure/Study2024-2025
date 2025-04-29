"""
Raspberry Pi Pico - Advanced Program 1: Data Logger
==================================================

This program demonstrates how to create a data logger that records
sensor readings to a file on the Pico's filesystem with timestamps.

Hardware required:
- Raspberry Pi Pico
- DHT22 or DHT11 temperature and humidity sensor
- DS3231 RTC (Real-Time Clock) module
- microSD card module (SPI interface)
- 10K ohm resistor (for DHT sensor)
- Breadboard and jumper wires
- Optional: LED for status indication

Connections:
- DHT sensor:
  - Connect VCC to 3.3V
  - Connect GND to GND
  - Connect DATA to GPIO pin 15
  - Connect a 10K resistor between DATA and VCC (pull-up)

- DS3231 RTC module:
  - Connect VCC to 3.3V
  - Connect GND to GND
  - Connect SDA to GPIO pin 16 (I2C0 SDA)
  - Connect SCL to GPIO pin 17 (I2C0 SCL)

- microSD card module:
  - Connect VCC to 3.3V
  - Connect GND to GND
  - Connect MISO to GPIO pin 4 (SPI0 RX)
  - Connect MOSI to GPIO pin 7 (SPI0 TX)
  - Connect SCK to GPIO pin 6 (SPI0 SCK)
  - Connect CS to GPIO pin 5 (SPI0 CSn)

- Optional: Connect an LED to GPIO pin 25 through a 220 ohm resistor

Note: You'll need to install the following libraries:
- dht.py for the DHT sensor
- ds3231.py for the RTC module
- sdcard.py for the microSD card

Instructions:
1. Set up the circuit as described above
2. Upload the required libraries to your Pico
3. Upload this program to the Pico
4. The program will log temperature and humidity data to a CSV file
5. You can remove the microSD card and view the data on a computer

Concepts covered:
- File I/O operations
- Real-time clock integration
- SD card interfacing with SPI
- Structured data storage (CSV format)
- Error handling and recovery
- Power-efficient logging with sleep
"""

import machine
import time
import os
import utime
import gc

# Import required libraries
try:
    import dht
    import ds3231
    import sdcard
except ImportError:
    print("Error: Required libraries not found")
    print("Please upload dht.py, ds3231.py, and sdcard.py to your Pico")
    raise

# Set up the DHT sensor
dht_pin = machine.Pin(15)
dht_sensor = dht.DHT22(dht_pin)  # Change to DHT11 if using that sensor

# Set up the I2C for the RTC
i2c = machine.I2C(0, sda=machine.Pin(16), scl=machine.Pin(17), freq=400000)

# Set up the RTC
try:
    rtc = ds3231.DS3231(i2c)
except Exception as e:
    print(f"Error initializing RTC: {e}")
    raise

# Set up the SPI for the SD card
spi = machine.SPI(0,
                 baudrate=1000000,
                 polarity=0,
                 phase=0,
                 bits=8,
                 firstbit=machine.SPI.MSB,
                 sck=machine.Pin(6),
                 mosi=machine.Pin(7),
                 miso=machine.Pin(4))

# Set up the SD card
sd_cs = machine.Pin(5, machine.Pin.OUT)
try:
    sd = sdcard.SDCard(spi, sd_cs)
    # Mount the SD card
    os.mount(sd, '/sd')
    print("SD card mounted successfully")
except Exception as e:
    print(f"Error mounting SD card: {e}")
    raise

# Set up the status LED
led = machine.Pin(25, machine.Pin.OUT)

# Function to blink the LED
def blink_led(times=1, delay=0.1):
    for _ in range(times):
        led.value(1)
        time.sleep(delay)
        led.value(0)
        time.sleep(delay)

# Function to get a timestamp from the RTC
def get_timestamp():
    try:
        dt = rtc.datetime()
        return "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
            dt[0], dt[1], dt[2], dt[4], dt[5], dt[6])
    except Exception as e:
        print(f"Error getting timestamp: {e}")
        # Return a fallback timestamp using the Pico's internal timer
        year, month, day, hour, minute, second, _, _ = utime.localtime()
        return "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
            year, month, day, hour, minute, second)

# Function to read sensor data
def read_sensor():
    try:
        dht_sensor.measure()
        temperature = dht_sensor.temperature()
        humidity = dht_sensor.humidity()
        return temperature, humidity
    except Exception as e:
        print(f"Error reading sensor: {e}")
        return None, None

# Function to log data to a file
def log_data(timestamp, temperature, humidity):
    # Create a filename based on the current date
    date_str = timestamp.split()[0]  # Extract the date part
    filename = f"/sd/log_{date_str}.csv"
    
    # Check if the file exists
    file_exists = False
    try:
        os.stat(filename)
        file_exists = True
    except OSError:
        pass
    
    try:
        # Open the file in append mode
        with open(filename, "a") as f:
            # Write header if the file is new
            if not file_exists:
                f.write("Timestamp,Temperature (C),Humidity (%)\n")
            
            # Write the data
            f.write(f"{timestamp},{temperature:.1f},{humidity:.1f}\n")
        
        print(f"Data logged: {timestamp}, Temp: {temperature:.1f}°C, Humidity: {humidity:.1f}%")
        blink_led(2, 0.1)  # Blink twice to indicate successful logging
        return True
    except Exception as e:
        print(f"Error logging data: {e}")
        blink_led(5, 0.1)  # Blink 5 times to indicate error
        return False

# Function to set the RTC if it's not already set
def check_and_set_rtc():
    dt = rtc.datetime()
    # Check if the RTC has a reasonable date (after 2020)
    if dt[0] < 2020:
        print("RTC not set. Setting to a default time.")
        # Set to a default time (you should set this to the correct time)
        rtc.datetime((2023, 1, 1, 0, 0, 0, 0, 0))
        print("RTC set to default: 2023-01-01 00:00:00")

# Main function
def main():
    print("Data Logger Started")
    print("------------------")
    
    # Check and set the RTC if needed
    check_and_set_rtc()
    
    # Log interval in seconds
    log_interval = 60  # Log every minute
    
    try:
        while True:
            # Read sensor data
            temperature, humidity = read_sensor()
            
            if temperature is not None and humidity is not None:
                # Get timestamp
                timestamp = get_timestamp()
                
                # Log the data
                log_data(timestamp, temperature, humidity)
            else:
                print("Failed to read sensor data")
                blink_led(3, 0.2)  # Blink 3 times to indicate sensor error
            
            # Free up memory
            gc.collect()
            
            # Wait for the next logging interval
            print(f"Waiting {log_interval} seconds until next log...")
            time.sleep(log_interval)
            
    except KeyboardInterrupt:
        print("\nData logging stopped by user")
    finally:
        # Clean up
        try:
            os.umount('/sd')
            print("SD card unmounted safely")
        except:
            pass
        led.value(0)
        print("Data logger shutdown complete")

# Run the main function
if __name__ == "__main__":
    main()