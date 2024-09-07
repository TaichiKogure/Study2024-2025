import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import time

plt.style.use('default')


def plot_data():
    plt.figure()

    while True:
        data = pd.read_csv("../BedRoomEnv.csv")

        # assuming the csv file has these columns, if different please adjust
        # Converting string date to datetime
        data['current_time'] = pd.to_datetime(data['current_time'])
        data.set_index('current_time', inplace=True)

        # clear the figure each time before drawing new data
        plt.clf()

        # creating subplots for each data type
        ax1 = plt.subplot(5, 1, 1)
        plt.plot(data.index, data['CO2'], label='CO2', color='c')
        ax1.xaxis.set_major_formatter(mdates.DateFormatter('%Y-%m-%d %H:%M:%S'))
        plt.ylabel('CO2')
        plt.grid(True)

        ax2 = plt.subplot(5, 1, 2, sharex=ax1)
        plt.plot(data.index, data['Tempereture'], label='Tempereture', color='r')
        plt.ylabel('Tempereture')
        plt.grid(True)

        ax3 = plt.subplot(5, 1, 3, sharex=ax1)
        plt.plot(data.index, data['Humidity'], label='Humidity', color='g')
        plt.ylabel('Humidity')
        plt.grid(True)

        ax4 = plt.subplot(5, 1, 4, sharex=ax1)
        plt.plot(data.index, data['Pressure'], label='Pressure', color='b')
        plt.ylabel('Pressure')
        plt.grid(True)

        ax5 = plt.subplot(5, 1, 5, sharex=ax1)
        plt.plot(data.index, data['GasResistance'], label='GasResistance', color='m')
        ax5.xaxis.set_major_formatter(mdates.DateFormatter('%Y-%m-%d %H:%M:%S'))
        plt.ylabel('GasResistance')
        plt.grid(True)

        # Adjust layout to ensure tick labels on different subplots do not overlap
        plt.tight_layout()

        plt.draw()
        plt.pause(60)  # Pause for 60 seconds


plot_data()
