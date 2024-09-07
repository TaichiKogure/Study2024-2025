#Recieve temp and Press data from RaspPiPicoW and output csv data

from flask import Flask, request
import csv
from datetime import datetime
import matplotlib.pyplot as plt


app = Flask(__name__)


@app.route('/')
def hello():
    hello = "Hello world"
    return hello


@app.route('/data', methods=['POST'])
def handle_data():
    data = request.json
    print(data)

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    data_1 = str(data.get('pressure')[:-3])
    data_2 = str(data.get('temperature')[:-1])
    row_data = {"current_time": timestamp, "Pressure": data_1, "Tempereture": data_2}

    with open('../PicodataX.csv', 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=["current_time", "Pressure", "Tempereture"])
        if f.tell() == 0:
            writer.writeheader()
        writer.writerow(row_data)

    return 'OK', 200


@app.route('/data2', methods=['POST'])
def handle_data2():
    data = request.json
    print(data)

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

    return 'OK', 200

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8888)
