# forPICO

import dht
from machine import Pin
import time

# prepare object and input DHT11 assined DataPIN
dht_sensor = dht.DHT11(Pin(21))

while True:
    try:
        # Operate sensing
        dht_sensor.measure()

        # Get Temp and Humidity
        temperature_celsius = dht_sensor.temperature()
        humidity_percentage = dht_sensor.humidity()

        # Print result
        print("TEMPERATURE: {:.1f}degC".format(temperature_celsius))
        print("HUMIDITY: {:.1f}%".format(humidity_percentage))

        # Stay 1sec
        time.sleep(1)

    except Exception as e:
        print("ERROR:", e)