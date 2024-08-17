from bme680 import *
from machine import I2C, Pin
import time
scl = Pin(19)
sda = Pin(18)
bme = BME680_I2C(I2C(1, scl=scl, sda=sda))

while True:
    print("Temp:{:.3g}C, Humidity:{:.3g}%, Pressure:{:.5g}hPa, Gas:{}".format(bme.temperature, bme.humidity, bme.pressure, bme.gas))
    time.sleep(3)