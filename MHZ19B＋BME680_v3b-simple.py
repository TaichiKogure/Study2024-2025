import csv
import time
import datetime
import os
import bme680
import mh_z19


def release_serial_port():
    os.system('sudo lsof /dev/serial0 | grep python | awk \'{print "kill -9", $2}\' | sh')


def read_co2():
    return mh_z19.read()["co2"]


def write_to_csv(current_time, co2_value, temp_press_hum_info):
    with open('data.csv', 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=["current_time", "co2_value", "temperature", "pressure", "humidity",
                                               "gas_resistance"])
        if f.tell() == 0:
            writer.writeheader()

        data_row = {"current_time": current_time, "co2_value": co2_value}
        data_row.update(temp_press_hum_info)

        writer.writerow(data_row)


def temperature_pressure_humidity(sensor):
    info = {}
    if sensor.get_sensor_data():
        info['temperature'] = sensor.data.temperature
        info['pressure'] = sensor.data.pressure
        info['humidity'] = sensor.data.humidity
        if sensor.data.heat_stable:
            info['gas_resistance'] = sensor.data.gas_resistance

    return info


if __name__ == '__main__':
    try:
        sensor = bme680.BME680(bme680.I2C_ADDR_PRIMARY)
    except (RuntimeError, IOError):
        sensor = bme680.BME680(bme680.I2C_ADDR_SECONDARY)

    sensor.set_humidity_oversample(bme680.OS_2X)
    sensor.set_pressure_oversample(bme680.OS_4X)
    sensor.set_temperature_oversample(bme680.OS_8X)
    sensor.set_filter(bme680.FILTER_SIZE_3)
    sensor.set_gas_status(bme680.ENABLE_GAS_MEAS)

    sensor.set_gas_heater_temperature(320)
    sensor.set_gas_heater_duration(150)
    sensor.select_gas_heater_profile(0)

    while True:
        release_serial_port()
        co2_value = read_co2()

        temp_press_hum_info = temperature_pressure_humidity(sensor)

        current_time = datetime.datetime.now()
        write_to_csv(current_time, co2_value, temp_press_hum_info)

        print(f'CO2 Value: {co2_value}')
        print(temp_press_hum_info)

        time.sleep(60)
