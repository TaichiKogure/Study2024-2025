from machine import Pin, I2C, ADC, UART
import network
from network import WLAN
from time import sleep
import urequests
import ujson
import gc
import onewire, ds18x20
import dht

# Initialize UART for MH-Z19 sensor
uart = UART(0, baudrate=9600, tx=Pin(16), rx=Pin(17))

# Initialize ADC for MQ-2 sensor
adc = ADC(26)  # GP26ピンを使用

# Initialize OneWire for DS18B20 sensor
ow = onewire.OneWire(Pin(4))
ds = ds18x20.DS18X20(ow)
roms = ds.scan()
print("Found DS devices: ", roms)

# Initialize DHT11 sensor
dht_pin = Pin(0)
dht11 = dht.DHT11(dht_pin)

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


def read_mh_z19():
    # Read command to get CO2 concentration
    uart.write(b'\xFF\x01\x86\x00\x00\x00\x00\x00\x79')
    sleep(0.1)  # Wait for the sensor to process the command

    if uart.any():
        response = uart.read(9)
        if response is not None and len(response) == 9 and response[0] == 0xFF and response[1] == 0x86:
            co2 = response[2] << 8 | response[3]
            return co2
    return None


def read_mq_2():
    analog_value = adc.read_u16()  # 16ビットのアナログ値を取得
    voltage = analog_value * (3.3 / 65535)  # 電圧に変換
    return analog_value, voltage


def read_temperature():
    ds.convert_temp()
    sleep(0.75)  # センサが温度変換を行うのを待つ
    for rom in roms:
        temperature = ds.read_temp(rom)
        return temperature
    return None


def read_dht11():
    try:
        dht11.measure()  # データを取得
        temperature = dht11.temperature()  # 温度を取得
        humidity = dht11.humidity()  # 湿度を取得
        return temperature, humidity
    except Exception as e:
        print("Failed to read from DHT11 sensor: ", e)
        return None, None


while True:
    try:
        # Read MH-Z19 sensor data
        co2 = read_mh_z19()
        if co2 is not None:
            print("CO2 Concentration: {} ppm".format(co2))
        else:
            print("Failed to read from MH-Z19 sensor")

        # Read MQ-2 sensor data
        analog_value, voltage = read_mq_2()
        print("MQ-2 Sensor Readings: Analog Value: {}, Voltage: {:.2f}V".format(analog_value, voltage))

        # Read DS18B20 sensor data
        temperature_ds18b20 = read_temperature()
        if temperature_ds18b20 is not None:
            print("DS18B20 Temperature: {:.2f}°C".format(temperature_ds18b20))
        else:
            print("Failed to read from DS18B20 sensor")

        # Read DHT11 sensor data
        temp_dht11, humid_dht11 = read_dht11()
        if temp_dht11 is not None and humid_dht11 is not None:
            print("DHT11 Sensor Readings: Temperature: {:.1f}°C, Humidity: {:.1f}%".format(temp_dht11, humid_dht11))

        # Aggregate data and send to the server
        data = {
            "mh_z19": {
                "co2": co2
            },
            "mq_2": {
                "analog_value": analog_value,
                "voltage": voltage
            },
            "ds18b20": {
                "temperature": temperature_ds18b20
            },
            "dht11": {
                "temperature": temp_dht11,
                "humidity": humid_dht11
            }
        }

        headers = {'content-type': 'application/json'}
        try:
            response = urequests.post(
                'http://192.168.3.47:8888/data4',
                headers=headers,
                data=ujson.dumps(data)
            )
            response.close()
        except Exception as e:
            print(e)

    except Exception as e:
        print('An error occurred:', e)

    sleep(1)
    gc.collect()
