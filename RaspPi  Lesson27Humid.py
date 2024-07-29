import time
import board
import adafruit_dht
makerobo_dhtDevice = adafruit_dht.DHT11(board.D16)

def loop():
    while True:
        try:
            TEMP_C_with_Humidity = makerobo_dhtDevice.temperature
            humidity = makerobo_dhtDevice.humidity
            print("Temperature on Sensor3 : {:.f} C Humidity:{}% ".format(TEMP_C_with_Humidity, humidity))
        except RuntimeError as error:
            print(error.args[0])
            time.sleep(2)
            continue

        except Exception as error:
            makerobo_dhtDevice.exit()
            raise error
        time.sleep(2)

if __name__ == '__main__':
    try:
        while True:
            loop()
    except KeyboardInterrupt:
        pass