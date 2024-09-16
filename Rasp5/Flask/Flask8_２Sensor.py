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

    # BME280のデータ処理
    pressure = data.get('pressure')
    temperature = data.get('temperature')
    row_data = {
        "current_time": timestamp,
        "Pressure": pressure,
        "Temperature": temperature
    }

    # CSVファイルに書き込み
    with open('PicodataX.csv', 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=["current_time", "Pressure", "Temperature"])
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

    # BME680のデータ処理
    out_temp = data.get('temperature')
    out_humidity = data.get('humidity')
    out_pressure = data.get('pressure')
    out_gas_res = data.get('gas_res')

    row_data = {
        "current_time": timestamp,
        "Temperature_outside": out_temp,
        "Humidity_outside": out_humidity,
        "Pressure_outside": out_pressure,
        "GasResistance_outside": out_gas_res
    }

    # CSVファイルに書き込み
    with open('OutsideEnv.csv', 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=row_data.keys())
        if f.tell() == 0:
            writer.writeheader()
        writer.writerow(row_data)

    processing_time = time.time() - start_time
    app.logger.info(f"Processing time: {processing_time} seconds")
    return 'OK', 200


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8888)
