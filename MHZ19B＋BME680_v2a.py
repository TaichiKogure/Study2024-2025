import csv
import time
import datetime
import os
import bme680
import mh_z19


def release_serial_port():
    os.system('sudo lsof /dev/serial0 | grep python | awk \'{print "kill -9", $2}\' | sh')


def read_co2():
    return str(mh_z19.read())[8:-1]


def write_to_csv(co2_value):
    current_time = datetime.datetime.now()

    with open('data.csv', 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([current_time, co2_value])


def temperature_pressure_humidity(sensor):
    info = {}
    if sensor.get_sensor_data():
        info['temperature'] = sensor.data.temperature
        info['pressure'] = sensor.data.pressure
        info['humidity'] = sensor.data.humidity

    return info


def indoor_air_quality(sensor, gas_baseline, burn_in_time):
    info = {}
    if sensor.get_sensor_data() and sensor.data.heat_stable:
        gas = sensor.data.gas_resistance
        gas_offset = gas_baseline - gas

        hum = sensor.data.humidity
        hum_offset = hum - hum_baseline

        # Calculate hum_score as the distance from the hum_baseline.
        if hum_offset > 0:
            hum_score = (100 - hum_baseline - hum_offset)
            hum_score /= (100 - hum_baseline)
            hum_score *= (hum_weighting * 100)

        else:
            hum_score = (hum_baseline + hum_offset)
            hum_score /= hum_baseline
            hum_score *= (hum_weighting * 100)

        # Calculate gas_score as the distance from the gas_baseline.
        if gas_offset > 0:
            gas_score = (gas / gas_baseline)
            gas_score *= (100 - (hum_weighting * 100))

        else:
            gas_score = 100 - (hum_weighting * 100)

        # Calculate air_quality_score.
        air_quality_score = hum_score + gas_score

        info['gas_resistance'] = gas
        info['humidity'] = hum
        info['air_quality'] = air_quality_score

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

    # Collect gas resistance burn-in values
    gas_baseline = sum(burn_in_data[-50:]) / 50.0
    hum_baseline = 40.0
    hum_weighting = 0.25

    while True:
        release_serial_port()
        co2_value = read_co2()
        write_to_csv(co2_value)
        print(f'CO2 Value: {co2_value}')

        temp_press_hum_info = temperature_pressure_humidity(sensor)
        print(temp_press_hum_info)

        indoor_air_info = indoor_air_quality(sensor, gas_baseline, burn_in_time)
        print(indoor_air_info)

        time.sleep(60)
