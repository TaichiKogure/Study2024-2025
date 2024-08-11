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

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    data_1 = float(data.get('pressure')[:-3])
    data_2 = float(data.get('tempereteure')[:-1])
    row_data = [data_1, data_2, timestamp]

    with open('storydata.csv', 'a') as f:
        writer = csv.writer(f)
        writer.writerow(row_data)

    df = pd.read_csv('storydata.csv', header=None)
    df[2] = pd.to_datetime(df[2])  # assuming timestamp is the third column

    plt.figure(figsize=(12, 6))


df[2] = pd.to_datetime(df[2])  # convert column to datetime if not done already
df = df.sort_values(by=2)  # sort dataframe by datetime
plt.plot(df[2], df[0], label='First column data')
plt.plot(df[2], df[1], label='Second column data')
plt.xlabel('Time')
plt.ylabel('Value')
plt.title('First and Second columns data over time')
plt.legend()
plt.savefig('first_second_data.png')

    return 'OK', 200


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8888)
