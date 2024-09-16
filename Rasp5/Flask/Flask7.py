from flask import Flask, request, jsonify
import csv
from datetime import datetime
import logging
import time

app = Flask(__name__)

# ログ設定
logging.basicConfig(level=logging.DEBUG)


@app.route('/')
def hello():
    hello = "Hello world"
    return hello


@app.route('/data', methods=['POST'])
def handle_data():
    start_time = time.time()
    data = request.json
    app.logger.info(f"Received data: {data}")

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    data_1 = str(data.get('pressure')[:-3])
    data_2 = str(data.get('temperature')[:-1])
    row_data = {"current_time": timestamp, "Pressure": data_1, "Tempereture": data_2}

    with open('../PicodataX.csv', 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=["current_time", "Pressure", "Tempereture"])
        if f.tell() == 0:
            writer.writeheader()
        writer.writerow(row_data)

    processing_time = time.time() - start_time
    app.logger.info(f"Processing time: {processing_time} seconds")
    return 'OK', 200


@app.route('/data2', methods=['POST'])
def handle_data2():
    start_time = time.time()
    data = request.json
    app.logger.info(f"Received data: {data}")

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    co2_concentration = str(data.get('co2', ''))
    temperature = str(data.get('temperature', ''))
    humidity = str(data.get('humidity', ''))
    pressure = str(data.get('pressure', ''))
    gas_resistance = str(data.get('gas_res', ''))

    row_data = {
        "current_time": timestamp,
        "CO2": co2_concentration,
        "Tempereture": temperature,
        "Humidity": humidity,
        "Pressure": pressure,
        "GasResistance": gas_resistance,
    }

    with open('../BedRoomEnv.csv', 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=row_data.keys())
        if f.tell() == 0:
            writer.writeheader()
        writer.writerow(row_data)

    processing_time = time.time() - start_time
    app.logger.info(f"Processing time: {processing_time} seconds")
    return 'OK', 200


@app.route('/data3', methods=['POST'])
def handle_data3():
    start_time = time.time()
    data = request.json
    app.logger.info(f"Received data: {data}")

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    out_temp = str(data.get('temperature', ''))
    out_humidity = str(data.get('humidity', ''))
    out_pressure = str(data.get('pressure', ''))
    out_GasR = str(data.get('gas_res', ''))

    row_data = {
        "current_time": timestamp,
        "Temperature-outside": out_temp,
        "Humidity-outside": out_humidity,
        "Pressure-outside": out_pressure,
        "GasResistance-outside": out_GasR,
    }

    with open('../OutsideEnv.csv', 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=row_data.keys())
        if f.tell() == 0:
            writer.writeheader()
        writer.writerow(row_data)

    processing_time = time.time() - start_time
    app.logger.info(f"Processing time: {processing_time} seconds")
    return 'OK', 200


@app.route('/data4', methods=['POST'])
def handle_data4():
    start_time = time.time()
    data = request.json
    app.logger.info(f"Received data: {data}")

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    co2 = str(data.get('mh_z19', {}).get('co2', ''))
    analog_value = str(data.get('mq_2', {}).get('analog_value', ''))
    voltage = str(data.get('mq_2', {}).get('voltage', ''))
    temperature_ds18b20 = str(data.get('ds18b20', {}).get('temperature', ''))
    temp_dht11 = str(data.get('dht11', {}).get('temperature', ''))
    humid_dht11 = str(data.get('dht11', {}).get('humidity', ''))

    row_data = {
        "current_time": timestamp,
        "CO2": co2,
        "AnalogValue": analog_value,
        "Voltage": voltage,
        "Temperature_DS18B20": temperature_ds18b20,
        "Temperature_DHT11": temp_dht11,
        "Humidity_DHT11": humid_dht11
    }

    with open('../LR_env.csv', 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=row_data.keys())
        if f.tell() == 0:
            writer.writeheader()
        writer.writerow(row_data)

    processing_time = time.time() - start_time
    app.logger.info(f"Processing time: {processing_time} seconds")
    return 'OK', 200


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8888)
