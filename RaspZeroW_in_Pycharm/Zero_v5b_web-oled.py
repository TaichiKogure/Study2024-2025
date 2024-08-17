import os
import time
import datetime
import bme680
import mh_z19
import requests
import csv
from Adafruit_SSD1306 import SSD1306_128_64  # Import the library for the display
from PIL import Image, ImageDraw, ImageFont  # Import additional libraries for drawing


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
            'http://192.168.3.47:8888/data2',
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

    # Initialize the display
    disp = SSD1306_128_64(rst=None)
    disp.begin()
    disp.clear()
    disp.display()

    # Create blank image for drawing
    width = disp.width
    height = disp.height
    image = Image.new('1', (width, height))

    # Get drawing object to draw on image
    draw = ImageDraw.Draw(image)

    # Load default font
    font = ImageFont.load_default()

    while True:
        release_serial_port()
        co2_value = read_co2()

        temp_press_hum_info = temperature_pressure_humidity(sensor)

        current_time = datetime.datetime.now().isoformat()
        post_data(co2_value, temp_press_hum_info, current_time)
        write_to_csv(current_time, co2_value, temp_press_hum_info)

        print(f'CO2 Value: {co2_value}')
        print(temp_press_hum_info)

        # Draw on the display
        draw.rectangle((0, 0, width, height), outline=0, fill=0)  # Clear screen
        draw.text((0, 0), time.strftime("%Y-%m-%d"), font=font, fill=255)  # Date in yellow
        draw.text((0, 10), time.strftime("%H:%M:%S"), font=font, fill=255)  # Time in yellow
        draw.text((0, 20), f"Temp: {temp_press_hum_info.get('temperature', 'N/A')} C", font=font, fill=1)  # blue text
        draw.text((0, 30), f"Pres: {temp_press_hum_info.get('pressure', 'N/A')} hPa", font=font, fill=1)  # blue text
        draw.text((0, 40), f"Hum: {temp_press_hum_info.get('humidity', 'N/A')} %", font=font, fill=1)  # blue text
        draw.text((0, 50), f"Gas: {temp_press_hum_info.get('gas_res', 'N/A')} Ohms", font=font, fill=1)  # blue text
        draw.text((0, 60), f"CO2: {co2_value} ppm", font=font, fill=1)  # blue text

        disp.image(image)
        disp.display()

        time.sleep(60)
