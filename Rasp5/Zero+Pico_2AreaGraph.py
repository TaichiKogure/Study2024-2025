import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import time

plt.style.use('default')


def plot_data(start_time=None, y_scales=None):
    plt.figure()

    while True:
        # Read data from CSV files
        bed_data = pd.read_csv("BedRoomEnv.csv")
        out_data = pd.read_csv("OutsideEnv.csv")

        # Converting string date to datetime
        bed_data['current_time'] = pd.to_datetime(bed_data['current_time'])
        bed_data.set_index('current_time', inplace=True)

        out_data['current_time'] = pd.to_datetime(out_data['current_time'])
        out_data.set_index('current_time', inplace=True)

        # clear the figure each time before drawing new data
        plt.clf()

        # Determine the x-axis limits
        x_start = pd.to_datetime(start_time) if start_time else bed_data.index.min()
        x_end = bed_data.index.max()

        # creating subplots for each data type
        ax1 = plt.subplot(5, 1, 1)
        plt.plot(bed_data.index, bed_data['CO2'], label='BedRoom CO2', color='c')
        ax1.xaxis.set_major_formatter(mdates.DateFormatter('%Y-%m-%d %H:%M:%S'))
        plt.xlim(x_start, x_end)
        plt.ylabel('CO2')
        plt.grid(True)
        if y_scales and 'CO2' in y_scales:
            ax1.set_ylim(y_scales['CO2'])

        ax2 = plt.subplot(5, 1, 2, sharex=ax1)
        plt.plot(bed_data.index, bed_data['Tempereture'], label='BedRoom Temperature', color='r')
        plt.plot(out_data.index, out_data['Temperature-outside'], label='Outside Temperature', color='r',
                 linestyle='dashed')
        plt.ylabel('Temperature')
        plt.grid(True)
        if y_scales and 'Temperature' in y_scales:
            ax2.set_ylim(y_scales['Temperature'])

        ax3 = plt.subplot(5, 1, 3, sharex=ax1)
        plt.plot(bed_data.index, bed_data['Humidity'], label='BedRoom Humidity', color='g')
        plt.plot(out_data.index, out_data['Humidity-outside'], label='Outside Humidity', color='g', linestyle='dashed')
        plt.ylabel('Humidity')
        plt.grid(True)
        if y_scales and 'Humidity' in y_scales:
            ax3.set_ylim(y_scales['Humidity'])

        ax4 = plt.subplot(5, 1, 4, sharex=ax1)
        plt.plot(bed_data.index, bed_data['Pressure'], label='BedRoom Pressure', color='b')
        plt.plot(out_data.index, out_data['Pressure-outside'], label='Outside Pressure', color='b', linestyle='dashed')
        plt.ylabel('Pressure')
        plt.grid(True)
        if y_scales and 'Pressure' in y_scales:
            ax4.set_ylim(y_scales['Pressure'])

        ax5 = plt.subplot(5, 1, 5, sharex=ax1)
        plt.plot(bed_data.index, bed_data['GasResistance'], label='BedRoom GasResistance', color='m')
        plt.plot(out_data.index, out_data['GasResistance-outside'], label='Outside GasResistance', color='m',
                 linestyle='dashed')
        ax5.xaxis.set_major_formatter(mdates.DateFormatter('%Y-%m-%d %H:%M:%S'))
        plt.xlim(x_start, x_end)
        plt.ylabel('GasResistance')
        plt.grid(True)
        if y_scales and 'GasResistance' in y_scales:
            ax5.set_ylim(y_scales['GasResistance'])

        # Adding legends for all subplots
        ax1.legend(loc='best')
        ax2.legend(loc='best')
        ax3.legend(loc='best')
        ax4.legend(loc='best')
        ax5.legend(loc='best')

        # Adjust layout to ensure tick labels on different subplots do not overlap
        plt.tight_layout()

        plt.draw()
        plt.pause(60)  # Pause for 60 seconds


plot_data(start_time="2023-01-01 00:00:00",
          y_scales={'CO2': (0, 1000), 'Temperature': (15, 30), 'Humidity': (20, 80), 'Pressure': (950, 1050),
                    'GasResistance': None})
