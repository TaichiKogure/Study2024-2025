from machine import Pin, I2C
from time import sleep
import BME280
from lcd1602 import LCD

# Initialize I2C communication for BME280 sensor
i2c1 = I2C(0, scl=Pin(21), sda=Pin(20), freq=10000)

# Initialize I2C communication for LCD;
# Set SDA to pin 20, SCL to pin 21, and frequency to 400kHz
i2c2 = I2C(0, sda=Pin(18), scl=Pin(19), freq=400000)

# Create an LCD object for interfacing with the LCD1602 display
lcd = LCD(i2c2)

while True:
    try:
        # Initialize BME280 sensor
        bme = BME280.BME280(i2c=i2c1)

        # Read sensor data
        tempC = str(bme.temperature)
        pres = str(bme.pressure)

        # Print sensor readings
        print('Temperature: ', tempC)
        print('Pressure: ', pres)

        # Display the sensor readings on the LCD
        lcd.clear()
        lcd.message('Temp: ' + tempC + '\nPres: ' + pres)

    except Exception as e:
        # Handle any exceptions during sensor reading
        print('An error occurred:', e)

    sleep(5)
