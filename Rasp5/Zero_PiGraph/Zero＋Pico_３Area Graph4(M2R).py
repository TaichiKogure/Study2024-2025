import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

plt.style.use('default')

DATA_FILES = [
    ("/home/koguretaichi/Documents/Flask/BedRoomEnv.csv", 'current_time'),
    ("/home/koguretaichi/Documents/Flask/OutsideEnv.csv", 'current_time'),
    ("/home/koguretaichi/Documents/Pi5_EnvData.csv", 'datetime'),
    ("/home/koguretaichi/Documents/Flask/PicodataX.csv", 'current_time'),
    ("/home/koguretaichi/Documents/Flask/LR_env.csv", 'current_time')
]


def load_data(file_path, date_column):
    """
    Load data from a CSV file and set the specified column as datetime index.
    """
    data = pd.read_csv(file_path)
    data[date_column] = pd.to_datetime(data[date_column])
    data.set_index(date_column, inplace=True)
    return data


def resample_data(data, rule='min', reference_index=None):
    """
    Resample the data to a consistent frequency.
    If reference_index is provided, align data to this index.
    """
    resampled_data = data.resample(rule).mean()
    if reference_index is not None:
        resampled_data = resampled_data.reindex(reference_index, method='nearest')
    return resampled_data


def plot_lines(ax, x_data, y_data_dict, labels, colors, linestyles, ylabel, y_scale=None):
    """
    Plot multiple lines on a single subplot.
    """
    for y_data, label, color, linestyle in zip(y_data_dict, labels, colors, linestyles):
        ax.plot(x_data, y_data_dict[y_data], label=label, color=color, linestyle=linestyle)
    ax.set_ylabel(ylabel)
    if y_scale:
        ax.set_ylim(y_scale)
    ax.grid(True)
    ax.legend(loc='best')


def create_subplot(index, row, x_data, y_data_dict, labels, colors, linestyles, ylabel, y_scale=None, sharex=None):
    ax = plt.subplot(5, 1, index, sharex=sharex)
    plot_lines(ax, x_data, y_data_dict, labels, colors, linestyles, ylabel, y_scale)
    if index == row:
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%Y-%m-%d %H:%M:%S'))
    return ax


def plot_data(start_time=None, y_scales=None):
    plt.figure()

    # Read and preprocess data
    datasets = [load_data(file, date_col) for file, date_col in DATA_FILES]

    reference_index = datasets[0].index  # Use first dataset's index as reference

    bed_data, out_data, lroom_data, pico_data, lr_data = [resample_data(data, reference_index=reference_index) for data
                                                          in datasets]

    while True:
        plt.clf()
        x_start = pd.to_datetime(start_time) if start_time else bed_data.index.min()
        x_end = bed_data.index.max()

        ax1 = create_subplot(1, 5, bed_data.index, {
            'CO2': bed_data['CO2'],
            'CO2_LR': lr_data['CO2']
        },
                             ['BedRoom', 'DiningRoom'], ['c', 'k'], ['solid', 'solid'], 'CO2',
                             y_scales.get('CO2') if y_scales else None)

        ax2 = create_subplot(2, 5, bed_data.index, {
            'BedRoom Temperature': bed_data['Tempereture'],
            'Outside Temperature': out_data['Temperature-outside'],
            'L_temp1': lroom_data['temp_C_sensor1'],
            'Outside Temperature-2': pico_data['Tempereture'],
            'temp_C_Sensor2': lr_data['Temperature_DS18B20']
        },
                             ['BedRoom', 'Outside1', 'Desk', 'Outside2', 'DiningRoom Temp.'],
                             ['r', 'r', 'orange', 'blue', 'brown'],
                             ['solid', 'dashed', 'dotted', 'solid', 'dotted'],
                             'Temperature',
                             y_scales.get('Temperature') if y_scales else None, sharex=ax1)

        ax3 = create_subplot(3, 5, bed_data.index, {
            'BedRoom Humidity': bed_data['Humidity'],
            'Outside Humidity': out_data['Humidity-outside'],
            'Humid_LR': lr_data['Humidity_DHT11']
        },
                             ['BedRoom', 'Outside', 'Dining'],
                             ['g', 'g', 'cyan'],
                             ['solid', 'dashed', 'dashdot'],
                             'Humidity',
                             y_scales.get('Humidity') if y_scales else None, sharex=ax1)

        ax4 = create_subplot(4, 5, bed_data.index, {
            'BedRoom Pressure': bed_data['Pressure'],
            'Outside Pressure': out_data['Pressure-outside']
        },
                             ['BedRoom', 'Outside'],
                             ['b', 'b'],
                             ['solid', 'dashed'],
                             'Pressure',
                             y_scales.get('Pressure') if y_scales else None, sharex=ax1)

        ax5 = create_subplot(5, 5, bed_data.index, {
            'BedRoom GasResistance': bed_data['GasResistance'],
            'Outside GasResistance': out_data['GasResistance-outside'],
            'Gas_LR': lr_data['AnalogValue']
        },
                             ['BedRoom', 'Outside', 'DiningRoom'],
                             ['m', 'm', 'gray'],
                             ['solid', 'dashed', 'solid'],
                             'GasResistance',
                             y_scales.get('GasResistance') if y_scales else None, sharex=ax1)

        plt.xlim(x_start, x_end)
        plt.tight_layout()
        plt.draw()
        plt.pause(30)


plot_data(start_time="2024-08-21 22:30:00",
          y_scales={'CO2': (420, 1000), 'Temperature': (21, 40), 'Humidity': (30, 80), 'Pressure': (990, 1005),
                    'GasResistance': (10000, 150000)})
