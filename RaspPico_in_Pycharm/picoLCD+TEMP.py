from machine import Pin, I2C, ADC
from time import sleep
import BME280
from I2C_LCD import TWI_LCD

# Initialize I2C communication for BME280 sensor
i2c1 = I2C(0, scl=Pin(21), sda=Pin(20), freq=10000)

# Initialize I2C communication for LCD;
# Set SDA to pin 0, SCL to pin 1, and frequency to 400kHz
i2c2 = I2C(1, sda=Pin(18), scl=Pin(19), freq=400000)

# Create an LCD object for interfacing with the LCD1602 display
lcd = TWI_LCD(i2c2, 0x27)

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
        lcd.clr_home()
        lcd.goto_xy(1, 0)
        lcd.put_str('Temp: ' + tempC + '\nPres: ' + pres)

    except Exception as e:
        # Handle any exceptions during sensor reading
        print('An error occurred:', e)

    sleep(5)
