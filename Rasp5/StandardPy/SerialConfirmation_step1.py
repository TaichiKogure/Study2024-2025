import serial
import time

# シリアルポートの設定
ser = serial.Serial(
    port='/dev/serial0',  # 使用するデバイスに応じてポートを変更
    baudrate=9600,  # デバイスのボーレートに設定
    timeout=1
)


def test_serial_connection():
    if ser.isOpen():
        print("Serial port is open")
        ser.write(b'Hello, Serial!\n')
        time.sleep(1)
        response = ser.readline()
        print("Response: ", response.decode('utf-8'))
    else:
        print("Failed to open serial port")


if __name__ == "__main__":
    test_serial_connection()
    ser.close()
