from machine import Pin, I2C, ADC
import network
from network import WLAN
from time import sleep
import BME280
import urequests
import ujson
import gc

# Initialize I2C communication for BME280 sensor
i2c1 = I2C(0, scl=Pin(21), sda=Pin(20), freq=10000)


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
        bme = BME280.BME280(i2c=i2c1)

        # Read sensor data
        tempC = str(bme.temperature)
        pres = str(bme.pressure)

        # Print sensor readings
        print('Temperature: ', tempC)
        print('Pressure: ', pres)

        # Send data to the server
        my_data = {"temperature": tempC, "pressure": pres}
        headers = {'content-type': 'application/json'}
        try:
            response = urequests.post(
                'http://192.168.3.47:8888/data',
                headers=headers,
                data=ujson.dumps(my_data)
            )
            response.close()
        except Exception as e:
            print(e)

    except Exception as e:
        # Handle any exceptions during sensor reading
        print('An error occurred:', e)

    sleep(60)
    gc.collect()


