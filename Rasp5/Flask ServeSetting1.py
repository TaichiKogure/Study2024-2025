from flask import Flask, request
import csv
from datetime import datetime
import matplotlib.pyplot as plt
import pandas as pd

app = Flask(__name__)


@app.route('/')
def hello():
    hello = "Hello world"
    return hello


@app.route('/data', methods=['POST'])
def handle_data():
    data = request.json
    print(data)

    data['Timestamp'] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    with open('picodata.csv', 'a') as f:
        writer = csv.DictWriter(f, fieldnames=data.keys())
        writer.writerow(data)

    df = pd.read_csv('picodata.csv')
    df['Timestamp'] = pd.to_datetime(df['Timestamp'])

    plt.figure(figsize=(12, 6))
    plt.plot(df['Timestamp'], df['Pressure'], label='Pressure')
    plt.plot(df['Timestamp'], df['Temperature'], label='Temperature')
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.title('Pressure and Temperature over time')
    plt.legend()
    plt.savefig('pressure_temperature.png')

    return 'OK', 200


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8888)
