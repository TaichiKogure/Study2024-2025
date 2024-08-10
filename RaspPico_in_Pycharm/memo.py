from machine import Pin, I2C, ADC
from time import sleep
import BME280
from I2C_LCD import TWI_LCD

# Initialize I2C communication for BME280 sensor
i2c1 = I2C(0, scl=Pin(21), sda=Pin(20), freq=10000)

# Initialize I2C communication for LCD;
# Set SDA to pin 0, SCL to pin 1, and frequency to 400kHz
i2c2 = I2C(1, sda=Pin(18), scl=Pin(19), freq=40000)

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
        lcd.goto_xy(0, 0)
        lcd.put_str('T:' + tempC)
        lcd.goto_xy(0, 1)
        lcd.put_str('P:' + pres)

    except Exception as e:
        # Handle any exceptions during sensor reading
        print('An error occurred:', e)

    sleep(1)

# Step1：Wifiに接続する。Huawei-Mm,1122334455Gre?

from network import WLAN
import ntptime
import time
import csv
from uos import mount
from uos import umount
import os
import urequests
import ujson

# Step1: Connect to WiFi
wlan = WLAN(mode=WLAN.STA)
nets = wlan.scan()
for net in nets:
    if net.ssid == 'Huawei-Mm':
        print('Network found!')
        wlan.connect(net.ssid, auth=(net.sec, '1122334455Gre?'), timeout=5000)
        while not wlan.isconnected():
            time.sleep(0.1)
        break

while True:
    # ここで何らかのデータを生成または取得します
    my_data = {"temperature", tempC, "humidity", pres}
    headers = {'content-type': 'application/json'}
    try:
        response = urequests.post(
            'http://192.168.3.47:8888/data',
            headers=headers,
            data=ujson.dumps(my_data)
        )
    except Exception as e:
        print(e)

    time.sleep(5)

# Step2:smb://192.168.3.47に接続する。
# Step3:Share ディレクトリにUpdatePICO.csvを作成する。
#　Step4：UpdatePICO.csvに1分ごとに測定したtempC,presの値と日時データを記入する。


# Step 2: SMB connection (You need to use upy_smbus for this purpose, which might not be available)
# This step is complex under MicroPython due to unavailability of required SMB libraries
# Couldn't be implemented due to lack of library support in MicroPython

# # Step 3: Create UpdatePICO.csv file in Share directory (Assuming locally due to step 2's limitation)

# try:
#     mount("/flash", "/Share")
# except OSError:
#     pass
#
# os.chdir('/Share')
# f = open('UpdatePICO.csv', 'w')


# # Step 4: Write tempC, pres readings and datetime data to UpdatePICO.csv every minute

# while True:
#     ntptime.settime()  # update time to network time
#     timestamp = '{0}-{1}-{2} {3}:{4}:{5}'.format(*time.localtime())  # generate timestamp: yyyy-mm-dd hh:mm:ss
#     tempC = str(bme.temperature)
#     pres = str(bme.pressure)
#     writer = csv.writer(f)
#     writer.writerow([timestamp, tempC, pres])
#     time.sleep(60)  # wait for a minute

from machine import Pin, I2C, ADC
from network import WLAN
from time import sleep
import BME280
import ntptime
import time
import csv
from uos import mount
from uos import umount
import os
import urequests
import ujson
from I2C_LCD import TWI_LCD

# Initialize I2C communication for BME280 sensor
i2c1 = I2C(0, scl=Pin(21), sda=Pin(20), freq=10000)

# Initialize I2C communication for LCD;
i2c2 = I2C(1, sda=Pin(18), scl=Pin(19), freq=40000)

# Create an LCD object for interfacing with the LCD1602 display
lcd = TWI_LCD(i2c2, 0x27)

# Step1: Connect to WiFi
wlan = WLAN(mode=WLAN.STA)
nets = wlan.scan()
for net in nets:
    if net.ssid == 'Huawei-Mm':
        print('Network found!')
        wlan.connect(net.ssid, auth=(net.sec, '1122334455Gre?'), timeout=5000)
        while not wlan.isconnected():
            time.sleep(0.1)
        break

while True:
    try:
        # Initialize the BME280 sensor
        bme = BME280.BME280(i2c=i2c1)

        # Read sensor data
        tempC = str(bme.temperature)
        pres = str(bme.pressure)

        # Print sensor readings
        print('Temperature: ', tempC)
        print('Pressure: ', pres)

        # Display the sensor readings on the LCD
        lcd.clr_home()
        lcd.goto_xy(0, 0)
        lcd.put_str('T:' + tempC)
        lcd.goto_xy(0, 1)
        lcd.put_str('P:' + pres)

        # Send data to the server
        my_data = {"temperature": tempC, "pressure": pres}
        headers = {'content-type': 'application/json'}
        try:
            response = urequests.post(
                'http://192.168.3.47:8888/data',
                headers=headers,
                data=ujson.dumps(my_data)
            )
        except Exception as e:
            print(e)

    except Exception as e:
        # Handle any exceptions during sensor reading
        print('An error occurred:', e)

    sleep(1)

####
#もうちょい修正版
####
from machine import Pin, I2C, ADC
from network import WLAN
from time import sleep
import BME280
import urequests
import ujson
from I2C_LCD import TWI_LCD

# Initialize I2C communication for BME280 sensor
i2c1 = I2C(0, scl=Pin(21), sda=Pin(20), freq=10000)

# Initialize I2C communication for LCD;
i2c2 = I2C(1, sda=Pin(18), scl=Pin(19), freq=40000)

# Create an LCD object for interfacing with the LCD1602 display
lcd = TWI_LCD(i2c2, 0x27)

# Wi-FiのSSIDとパスワードを設定
ssid = 'Huawei-Mm'
password = '1122334455Gre?'

wlan = WLAN(mode=WLAN.STA)
wlan.active(True)
wlan.connect(ssid, password)

# 接続が確認できない場合は1秒おきに再確認する(最大10回)
max_wait = 10
while max_wait > 0:
    if wlan.status() < 0 or wlan.status() >= 3:
        break
    max_wait -= 1
    print('waiting for connection...')
    sleep(1)

# 接続できない場合はエラーを返す
if wlan.status() != 3:
    raise RuntimeError('network connection failed')
else:
    print('Connected')
    status = wlan.ifconfig()
    print('ip = ' + status[0])

while True:
    try:
        # Initialize the BME280 sensor
        bme = BME280.BME280(i2c=i2c1)

        # Read sensor data
        tempC = str(bme.temperature)
        pres = str(bme.pressure)

        # Print sensor readings
        print('Temperature: ', tempC)
        print('Pressure: ', pres)

        # Display the sensor readings on the LCD
        lcd.clr_home()
        lcd.goto_xy(0, 0)
        lcd.put_str('T:' + tempC)
        lcd.goto_xy(0, 1)
        lcd.put_str('P:' + pres)

        # Send data to the server
        my_data = {"temperature": tempC, "pressure": pres}
        headers = {'content-type': 'application/json'}
        try:
            response = urequests.post(
                'http://192.168.3.47:8888/data',
                headers=headers,
                data=ujson.dumps(my_data)
            )
        except Exception as e:
            print(e)

    except Exception as e:
        # Handle any exceptions during sensor reading
        print('An error occurred:', e)

    sleep(10)

# 切断
wlan.disconnect()
print('Disconnect')
