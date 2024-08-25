import machine
import time

# ADCの初期化
adc = machine.ADC(26)  # GP26ピンを使用


def read_mq_2():
    analog_value = adc.read_u16()  # 16ビットのアナログ値を取得
    voltage = analog_value * (3.3 / 65535)  # 電圧に変換
    return analog_value, voltage


def main():
    while True:
        analog_value, voltage = read_mq_2()
        print("Analog Value: {}, Voltage: {:.2f}V".format(analog_value, voltage))
        time.sleep(1)  # 1秒待つ


if __name__ == "__main__":
    main()
