import csv
import time
import matplotlib.pyplot as plt
import matplotlib.dates as md
import mh_z19
import datetime

import os

os.system('sudo lsof /dev/serial0 | grep python | awk \'{print "kill -9", $2}\' | sh')

def read():
    out = mh_z19.read
    return str(out)[8:-1]


if __name__ == '__main__':
    fig, ax = plt.subplots()
    ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))

    while True:
        co2 = read()
        print(co2)

        current_time = datetime.datetime.now()

        # Append the data to the csv file
        with open('data.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([current_time, co2])

        # Read the updated csv file
        data = []
        with open('data.csv', 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                data.append(row)

        # Split the data into time and co2 for plotting
        times = [datetime.datetime.strptime(row[0], '%Y-%m-%d %H:%M:%S.%f') for row in data]
        co2_values = [float(row[1]) for row in data]

        # Clear the previous plot
        plt.cla()

        # Plot the updated data
        ax.plot(times, co2_values)
        fig.autofmt_xdate()
        plt.draw()

        # Pause for 60 seconds
        plt.pause(60)
