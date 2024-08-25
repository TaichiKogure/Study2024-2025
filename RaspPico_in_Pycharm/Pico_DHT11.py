import machine
import time
import dht

# DHT11の初期化 (GP0ピンを使用)
dht_pin = machine.Pin(0)
dht11 = dht.DHT11(dht_pin)


def read_dht11():
    try:
        dht11.measure()  # データを取得
        temperature = dht11.temperature()  # 温度を取得
        humidity = dht11.humidity()  # 湿度を取得

        # 温度と湿度の出力を小数点第1位まで表示
        print("Temperature: {:.1f}°C".format(temperature))
        print("Humidity: {:.1f}%".format(humidity))
    except Exception as e:
        print("Failed to read from DHT11 sensor: ", e)


def main():
    while True:
        read_dht11()
        time.sleep(1)  # 1秒待つ


if __name__ == "__main__":
    main()
