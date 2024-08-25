import machine
import time

# UARTの初期化
uart = machine.UART(0, baudrate=9600, tx=machine.Pin(16), rx=machine.Pin(17))


def read_mh_z19():
    # Read command to get CO2 concentration
    uart.write(b'\xFF\x01\x86\x00\x00\x00\x00\x00\x79')
    time.sleep(0.1)  # Wait for the sensor to process the command

    if uart.any():
        response = uart.read(9)
        if response is not None and len(response) == 9 and response[0] == 0xFF and response[1] == 0x86:
            co2 = response[2] << 8 | response[3]
            return co2
    return None


def main():
    while True:
        co2 = read_mh_z19()
        if co2 is not None:
            print("CO2 Concentration: {} ppm".format(co2))
        else:
            print("Failed to read from MH-Z19 sensor")
        time.sleep(1)  # Wait for 1 second


if __name__ == "__main__":
    main()
