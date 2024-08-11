import os
import time
import datetime
import bme680
import mh_z19
import requests
import csv


def release_serial_port():
    os.system('sudo lsof /dev/serial0 | grep python | awk \'{print "kill -9", $2}\' | sh')


def read_co2():
    return mh_z19.read()["co2"]


def write_to_csv(current_time, co2_value, temp_press_hum_info):
    with open('BedRoomEnv.csv', 'a', newline='') as f:
        fieldnames = ["current_time", "temperature", "pressure", "humidity", "gas_res", "co2"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)

        if f.tell() == 0:
            writer.writeheader()

        data_row = {"current_time": current_time, "co2": co2_value, **temp_press_hum_info}
        writer.writerow(data_row)


def post_data(co2_value, temp_press_hum_info, current_time):
    data = temp_press_hum_info
    data["co2"] = co2_value
    data["current_time"] = current_time
    try:
        response = requests.post(
            'http://192.168.3.47:8888/data',
            headers={'content-type': 'application/json'},
            json=data
        )
        print('Post response:', response.text)
    except Exception as e:
        print(e)


def temperature_pressure_humidity(sensor):
    info = {}
    if sensor.get_sensor_data():
        info['temperature'] = round(sensor.data.temperature, 2)
        info['pressure'] = round(sensor.data.pressure, 2)
        info['humidity'] = round(sensor.data.humidity, 2)
        if sensor.data.heat_stable:
            info['gas_res'] = round(sensor.data.gas_resistance, 1)

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

        current_time = datetime.datetime.now().isoformat()
        post_data(co2_value, temp_press_hum_info, current_time)
        write_to_csv(current_time, co2_value, temp_press_hum_info)

        print(f'CO2 Value: {co2_value}')
        print(temp_press_hum_info)

        time.sleep(60)

######
### Old Data
######
# import csv
# import time
# import datetime
# import os
# import bme680
# import mh_z19
#
#
# def release_serial_port():
#     os.system('sudo lsof /dev/serial0 | grep python | awk \'{print "kill -9", $2}\' | sh')
#
#
# def read_co2():
#     return mh_z19.read()["co2"]
#
#
# def write_to_csv(current_time, co2_value, temp_press_hum_info):
#     with open('../Env_data.csv', 'a', newline='') as f:
#         writer = csv.DictWriter(f, fieldnames=["current_time", "co2_value", "temperature", "pressure", "humidity",
#                                                "gas_resistance"])
#         if f.tell() == 0:
#             writer.writeheader()
#
#         data_row = {"current_time": current_time, "co2_value": co2_value}
#         data_row.update(temp_press_hum_info)
#
#         writer.writerow(data_row)
#
#
# def temperature_pressure_humidity(sensor):
#     info = {}
#     if sensor.get_sensor_data():
#         info['temperature'] = round(sensor.data.temperature, 2)  # 小数点以下2桁まで丸める
#         info['pressure'] = round(sensor.data.pressure, 2)  # 小数点以下2桁まで丸める
#         info['humidity'] = round(sensor.data.humidity, 2)  # 小数点以下2桁まで丸める
#         if sensor.data.heat_stable:
#             info['gas_resistance'] = round(sensor.data.gas_resistance, 1)  # 小数点以下1桁まで丸める
#
#     return info
#
#
# if __name__ == '__main__':
#     try:
#         sensor = bme680.BME680(bme680.I2C_ADDR_PRIMARY)
#     except (RuntimeError, IOError):
#         sensor = bme680.BME680(bme680.I2C_ADDR_SECONDARY)
#
#     sensor.set_humidity_oversample(bme680.OS_2X)
#     sensor.set_pressure_oversample(bme680.OS_4X)
#     sensor.set_temperature_oversample(bme680.OS_8X)
#     sensor.set_filter(bme680.FILTER_SIZE_3)
#     sensor.set_gas_status(bme680.ENABLE_GAS_MEAS)
#
#     sensor.set_gas_heater_temperature(320)
#     sensor.set_gas_heater_duration(150)
#     sensor.select_gas_heater_profile(0)
#
#     while True:
#         release_serial_port()
#         co2_value = read_co2()
#
#         temp_press_hum_info = temperature_pressure_humidity(sensor)
#
#         current_time = datetime.datetime.now()
#         write_to_csv(current_time, co2_value, temp_press_hum_info)
#
#         print(f'CO2 Value: {co2_value}')
#         print(temp_press_hum_info)
#
#         time.sleep(60)

# 実施したいこと①以下の形式に合わせて、下記アドレスへPOSTする形に変更したい。
# {
#   "pressure": "xxxx",
#   "temperature": "xxxx",
#   "humidity": "xxxx",
#   "gas_res": "xxxx",
#   "co2": "xxxx"
# }
#         # Send data to the server
#         my_data = {"temperature": tempC, "pressure": pres}
#         headers = {'content-type': 'application/json'}
#         try:
#             response = urequests.post(
#                 'http://192.168.3.47:8888/data',
#                 headers=headers,
#                 data=ujson.dumps(my_data)
#             )
#         except Exception as e:
#             print(e)