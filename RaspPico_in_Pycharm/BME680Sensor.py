import machine
import time
import bme680

# I2Cの初期化
i2c = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0))

# BME680センサの初期化
sensor = bme680.BME680(i2c=i2c)

# センサの設定
sensor.set_humidity_oversample(bme680.OS_2X)
sensor.set_pressure_oversample(bme680.OS_4X)
sensor.set_temperature_oversample(bme680.OS_8X)
sensor.set_filter(bme680.FILTER_SIZE_3)
sensor.set_gas_status(bme680.ENABLE_GAS_MEAS)

# ガスヒータの設定
sensor.set_gas_heater_temperature(320)
sensor.set_gas_heater_duration(150)
sensor.select_gas_heater_profile(0)


def read_sensor():
    if sensor.get_sensor_data():
        temperature = sensor.data.temperature
        pressure = sensor.data.pressure
        humidity = sensor.data.humidity
        gas_resistance = sensor.data.gas_resistance

        print("Temperature: {:.2f} °C".format(temperature))
        print("Pressure: {:.2f} hPa".format(pressure))
        print("Humidity: {:.2f} %".format(humidity))
        print("Gas Resistance: {:.2f} Ohms".format(gas_resistance))
    else:
        print("Failed to retrieve data")


while True:
    read_sensor()  # 30秒ごと
    time.sleep(30)
