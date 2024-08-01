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


def write_to_csv(current_time, co2_value, temp_press_hum_info, indoor_air_info):
    with open('data.csv', 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=["current_time", "co2_value", "temperature", "pressure", "humidity",
                                               "gas_resistance", "air_quality"])
        if f.tell() == 0:
            writer.writeheader()

        data_row = {"current_time": current_time, "co2_value": co2_value}
        data_row.update(temp_press_hum_info)
        data_row.update(indoor_air_info)

        writer.writerow(data_row)


def temperature_pressure_humidity(sensor):
    info = {}
    if sensor.get_sensor_data():
        info['temperature'] = sensor.data.temperature
        info['pressure'] = sensor.data.pressure
        info['humidity'] = sensor.data.humidity

    return info


def indoor_air_quality(sensor, gas_baseline, hum_baseline, hum_weighting):
    info = {}
    if sensor.get_sensor_data() and sensor.data.heat_stable:
        gas = sensor.data.gas_resistance
        gas_offset = gas_baseline - gas

        hum = sensor.data.humidity
        hum_offset = hum - hum_baseline

        # Calculate hum_score as the distance from the hum_baseline.
        if hum_offset > 0:
            hum_score = (100 - hum_baseline - hum_offset) / (100 - hum_baseline) * (hum_weighting * 100)

        else:
            hum_score = (hum_baseline + hum_offset) / hum_baseline * (hum_weighting * 100)

        # Calculate gas_score as the distance from the gas_baseline.
        if gas_offset > 0:
            gas_score = (gas / gas_baseline) * (100 - (hum_weighting * 100))

        else:
            gas_score = 100 - (hum_weighting * 100)

        # Calculate air_quality_score.
        air_quality_score = hum_score + gas_score

        info['gas_resistance'] = gas
        info['air_quality'] = air_quality_score

    return info


def calc_gas_baseline(sensor, burn_in_time):
    gas_baseline = None
    burn_in_data = []

    start_time = time.time()
    curr_time = start_time

    # Collect gas resistance burn-in values
    while curr_time - start_time < burn_in_time:
        curr_time = time.time()
        if sensor.get_sensor_data() and sensor.data.heat_stable:
            gas = sensor.data.gas_resistance
            burn_in_data.append(gas)
            print('Gas: {0} Ohms'.format(gas), end="\r")
            time.sleep(1)

    if len(burn_in_data) >= 50:
        gas_baseline = sum(burn_in_data[-50:]) / 50.0

    return gas_baseline


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

    burn_in_time = 300
    hum_baseline = 40.0
    hum_weighting = 0.25

    gas_baseline = calc_gas_baseline(sensor, burn_in_time)
    if gas_baseline is None:
        print("Failed to calculate gas baseline")
        exit(1)

    print('\nGas baseline: {0} Ohms, humidity baseline: {1:.2f} %RH\n'.format(
        gas_baseline,
        hum_baseline))

    while True:
        release_serial_port()
        co2_value = read_co2()

        temp_press_hum_info = temperature_pressure_humidity(sensor)
        indoor_air_info = indoor_air_quality(sensor, gas_baseline, hum_baseline, hum_weighting)

        current_time = datetime.datetime.now()
        write_to_csv(current_time, co2_value, temp_press_hum_info, indoor_air_info)

        print(f'CO2 Value: {co2_value}')
        print(temp_press_hum_info)
        print(indoor_air_info)

        time.sleep(60)
