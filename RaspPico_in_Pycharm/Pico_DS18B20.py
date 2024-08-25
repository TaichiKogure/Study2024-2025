import machine
import time
import onewire, ds18x20

# OneWireバスの初期化 (GP4ピンを使用)
ow = onewire.OneWire(machine.Pin(4))
ds = ds18x20.DS18X20(ow)

# 接続されているすべてのデバイスを検索
roms = ds.scan()
print("Found DS devices: ", roms)


def read_temperature():
    ds.convert_temp()
    time.sleep_ms(750)  # センサが温度変換を行うのを待つ
    for rom in roms:
        temperature = ds.read_temp(rom)
        print("Temperature: {:.2f}°C".format(temperature))
        return temperature


def main():
    while True:
        read_temperature()
        time.sleep(1)  # 1秒待つ


if __name__ == "__main__":
    main()
