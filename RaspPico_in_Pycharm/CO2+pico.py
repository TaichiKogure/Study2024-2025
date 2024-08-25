from machine import Pin, I2C, UART
import dht
import onewire
import ds18x20
import network
from time import sleep
import urequests
import ujson
import gc

# Wi-FiのSSIDとパスワードを設定
ssid = 'Huawei-Mm'
password = '1122334455Gre?'

wlan = network.WLAN(network.STA_IF)
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

# Initialize sensors
uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))  # For MH-Z19 CO2 sensor
mq19 = ADC(Pin(36))  # For MQ-19 gas sensor (adapt pin as necessary)
dht11_pin = Pin(27)  # For DHT11 sensor
dht11 = dht.DHT11(dht11_pin)
ds_pin = Pin(18)  # For DS18B20 sensor
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))
roms = ds_sensor.scan()

while True:
    try:
        # Read MH-Z19 CO2 sensor data
        uart.write(b'\xFF\x01\x86\x00\x00\x00\x00\x00\x79')
        sleep(1)
        response = uart.read(9)
        if response is not None and len(response) == 9:
            co2 = response[2] * 256 + response[3]
        else:
            co2 = None

        # Read MQ-19 gas sensor data
        gas_level = mq19.read()  # Adjust as per MQ gas sensor reading method

        # Read DHT11 sensor data
        dht11.measure()
        humidity = dht11.humidity()
        temp_dht11 = dht11.temperature()

        # Read DS18B20 sensor data
        ds_sensor.convert_temp()
        sleep(1)
        temp_ds18b20 = ds_sensor.read_temp(roms[0])

        # Print sensor readings
        print('MH-Z19 CO2 Sensor Reading: ', co2)
        print('MQ-19 Gas Sensor Reading: ', gas_level)
        print('DHT11 Sensor Readings: Temperature: {}C Humidity: {}%'.format(temp_dht11, humidity))
        print('DS18B20 Sensor Reading: ', temp_ds18b20)

        # Send sensor data to the server
        data = {
            "CO2": co2,
            "MQ19_gas": gas_level,
            "DHT11_temperature": temp_dht11,
            "DHT11_humidity": humidity,
            "DS18B20_temperature": temp_ds18b20
        }
        headers = {'content-type': 'application/json'}
        try:
            response = urequests.post(
                'http://192.168.3.47:8888/data',
                headers=headers,
                data=ujson.dumps(data)
            )
            response.close()
        except Exception as e:
            print(e)

        sleep(5)  # Delay between readings

    except Exception as e:
        # Handle any exceptions during sensor reading
        print(e)
    gc.collect()  # Garbage collection to free memory
