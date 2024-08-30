from machine import Pin, I2C, ADC, UART, SPI
import network
from network import WLAN
from time import sleep
import urequests
import ujson
import gc
import onewire, ds18x20
import dht
import framebuf
import time

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
    sleep(5)  # Wait for the sensor to process the command and send the response

    response = uart.read(9)
    if response:
        if len(response) == 9:
            checksum = 0xFF - (sum(response[1:8]) & 0xFF)
            if response[0] == 0xFF and response[1] == 0x86 and response[8] == checksum:
                co2 = response[2] << 8 | response[3]
                return co2
            else:
                print("Checksum mismatch")
        else:
            print("Response length is incorrect")
    else:
        print("No response from sensor")

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


# SPIの初期化
DC_PIN = 13
RST_PIN = 12
CS_PIN = 5
spi = SPI(0, baudrate=500000, polarity=0, phase=0, sck=Pin(2), mosi=Pin(3))


# 初期化関数
def oled_init(dc_pin, rst_pin, cs_pin):
    dc = Pin(dc_pin, Pin.OUT)
    rst = Pin(rst_pin, Pin.OUT)
    cs = Pin(cs_pin, Pin.OUT)

    # SSD1309 ディスプレイの初期化コマンド
    init_cmds = [
        0xAE,  # Display off
        0xD5, 0x80,  # Set display clock division
        0xA8, 0x3F,  # Set multiplex ratio (1 to 64)
        0xD3, 0x00,  # Set display offset
        0x40,  # Set start line address
        0x8D, 0x14,  # Charge Pump
        0x20, 0x00,  # Memory mode
        0xA1,  # Segment re-map
        0xC8,  # COM output scan direction
        0xDA, 0x12,  # Set com pins hardware configuration
        0x81, 0xCF,  # Set contrast control register
        0xD9, 0xF1,  # Set pre-charge period
        0xDB, 0x40,  # Set vcomh
        0xA4,  # Output follows RAM content
        0xA6,  # Normal display
        0xAF  # Display on
    ]

    # リセット
    rst.value(0)
    time.sleep(0.01)
    rst.value(1)

    # コマンド送信
    for cmd in init_cmds:
        cs.value(0)
        dc.value(0)  # Command mode
        spi.write(bytearray([cmd]))
        cs.value(1)

    return dc, cs


# OLEDの初期化
dc, cs = oled_init(DC_PIN, RST_PIN, CS_PIN)

# フレームバッファ
width = 128
height = 64
buffer = bytearray(width * height // 8)
fb = framebuf.FrameBuffer(buffer, width, height, framebuf.MONO_VLSB)


# ディスプレイにデータを表示する関数
def display_data(co2, temp_ds18b20, analog_value, voltage, temp_dht11, humid_dht11):
    fb.fill(0)
    fb.text('CO2: {} ppm'.format(co2), 0, 0, 1)
    fb.text('DS18B20: {:.2f}C'.format(temp_ds18b20), 0, 10, 1)
    fb.text('MQ-2: {:.2f}V'.format(voltage), 0, 20, 1)
    fb.text('DHT11_T: {:.1f}C'.format(temp_dht11), 0, 30, 1)
    fb.text('DHT_H: {:.1f}%'.format(humid_dht11), 0, 40, 1)
    update_display()


# ディスプレイ情報を送信
def update_display():
    for page in range(0, 8):
        cs.value(0)
        dc.value(0)  # Command mode
        spi.write(bytearray([0xB0 + page]))  # Set page address
        spi.write(bytearray([0x00]))  # Set lower column address
        spi.write(bytearray([0x10]))  # Set higher column address
        cs.value(1)

        cs.value(0)
        dc.value(1)  # Data mode
        spi.write(buffer[page * 128:(page + 1) * 128])
        cs.value(1)


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

        # ディスプレイにデータを表示
        display_data(co2, temperature_ds18b20, analog_value, voltage, temp_dht11, humid_dht11)

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

    sleep(20)
    gc.collect()
