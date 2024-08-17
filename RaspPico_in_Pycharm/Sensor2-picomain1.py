from machine import Pin, I2C, ADC
import network
from network import WLAN
from time import sleep
import BME280
from bme680 import *
import urequests
import ujson
import gc

# Initialize I2C communication for BME280 sensor
i2c1 = I2C(0, scl=Pin(21), sda=Pin(20), freq=10000)

# Initialize I2C communication for BME680 sensor
scl = Pin(19)
sda = Pin(18)
i2c2 = I2C(1, scl=scl, sda=sda)
bme680 = bme680.BME680_I2C(i2c2)

# Wi-FiのSSIDとパスワードを設定
ssid = 'Huawei-Mm'
password = '1122334455Gre?'

wlan = WLAN(network.STA_IF)
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
        bme280 = BME280.BME280(i2c=i2c1)

        # Read BME280 sensor data
        tempC_280 = str(bme280.temperature)
        pres_280 = str(bme280.pressure)

        # Print BME280 sensor readings
        print('BME280 Sensor Readings:')
        print('Temperature: ', tempC_280)
        print('Pressure: ', pres_280)

        # Send BME280 data to the server
        data280 = {"temperature": tempC_280, "pressure": pres_280}
        headers = {'content-type': 'application/json'}
        try:
            response = urequests.post(
                'http://192.168.3.47:8888/data',
                headers=headers,
                data=ujson.dumps(data280)
            )
            response.close()
        except Exception as e:
            print(e)

        # Read BME680 sensor data
        tempC_680 = bme680.temperature
        hum_680 = bme680.humidity
        pres_680 = bme680.pressure
        gas_680 = bme680.gas

        # Print BME680 sensor readings
        print('BME680 Sensor Readings:')
        print(
            "Temp:{:.3g}C, Humidity:{:.3g}%, Pressure:{:.5g}hPa, Gas:{}".format(tempC_680, hum_680, pres_680, gas_680))

        # Send BME680 data to the server
        data680 = {
            "temperature": tempC_680,
            "humidity": hum_680,
            "pressure": pres_680,
            "gas_res": gas_680
        }
        try:
            response = urequests.post(
                'http://192.168.3.47:8888/data3',
                headers=headers,
                data=ujson.dumps(data680)
            )
            response.close()
        except Exception as e:
            print(e)

    except Exception as e:
        # Handle any exceptions during sensor reading
        print('An error occurred:', e)

    sleep(60)
    gc.collect()
