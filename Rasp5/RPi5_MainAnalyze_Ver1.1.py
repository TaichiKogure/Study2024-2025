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


def calculate_absolute_humidity(temp_c, relative_humidity):
    """Calculate Absolute Humidity (g/m^3)"""
    temp_k = temp_c + 273.15
    absolute_humidity = (6.112 * relative_humidity * 2.1674 * 10 ** 6) / temp_k
    return absolute_humidity


def calculate_comfort_index(temp_c, relative_humidity):
    """Calculate Comfort Index"""
    return temp_c - 0.55 * (1 - relative_humidity / 100) * (temp_c - 14.5)


def load_data(file_path, date_column):
    """
    Load data from a CSV file and set the specified column as datetime index.
    """
    data = pd.read_csv(file_path)
    data[date_column] = pd.to_datetime(data[date_column])
    data.set_index(date_column, inplace=True)
    print(f"Loaded data from {file_path}:")
    print(data.head())  # 先頭数行を確認
    return data


def resample_data(data, rule='min', reference_index=None):
    """
    Resample the data to a consistent frequency.
    If reference_index is provided, align data to this index.
    """
    resampled_data = data.resample(rule).mean()
    if reference_index is not None:
        resampled_data = resampled_data.reindex(reference_index, method='nearest')
    print(f"Resampled data:")
    print(resampled_data.head())  # 先頭数行を確認
    return resampled_data


def plot_lines(ax, x_data, y_data_dict, labels, colors, linestyles, ylabel, y_scale=None):
    """
    Plot multiple lines on a single subplot.
    """
    for y_data, label, color, linestyle in zip(y_data_dict, labels, colors, linestyles):
        print(f"Plotting {label} with color {color} and linestyle {linestyle}.")
        print(y_data_dict[y_data].head())  # 先頭数行を確認
        ax.plot(x_data, y_data_dict[y_data], label=label, color=color, linestyle=linestyle)
    ax.set_ylabel(ylabel)
    if y_scale:
        ax.set_ylim(y_scale)
    ax.grid(True)
    ax.legend(loc='best')


def create_subplot(index, row, x_data, y_data_dict, labels, colors, linestyles, ylabel, y_scale=None, sharex=None):
    ax = plt.subplot(3, 2, index, sharex=sharex)
    plot_lines(ax, x_data, y_data_dict, labels, colors, linestyles, ylabel, y_scale)
    if index >= 5:
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%Y-%m-%d %H:%M:%S'))
    return ax


def plot_data(start_time=None, y_scales=None):
    plt.figure()

    while True:
        # Read and preprocess data
        datasets = [load_data(file, date_col) for file, date_col in DATA_FILES]

        reference_index = datasets[0].index  # Use first dataset's index as reference

        bed_data, out_data, lroom_data, pico_data, lr_data = [resample_data(data, reference_index=reference_index) for
                                                              data in datasets]

        plt.clf()
        x_start = pd.to_datetime(start_time) if start_time else bed_data.index.min()
        x_end = bed_data.index.max()

        # Calculate Absolute Humidity
        bed_data['Absolute_Humidity'] = calculate_absolute_humidity(bed_data['Tempereture'], bed_data['Humidity'])
        out_data['Absolute_Humidity'] = calculate_absolute_humidity(out_data['Temperature-outside'],
                                                                    out_data['Humidity-outside'])
        lr_data['Absolute_Humidity'] = calculate_absolute_humidity(lr_data['Temperature_DS18B20'],
                                                                   lr_data['Humidity_DHT11'])

        ax1 = create_subplot(1, 6, bed_data.index, {
            'BedRoom Absolute Humidity': bed_data['Absolute_Humidity'],
            'Outside Absolute Humidity': out_data['Absolute_Humidity'],
            'LR Absolute Humidity': lr_data['Absolute_Humidity']
        },
                             ['BedRoom', 'Outside', 'DiningRoom'], ['c', 'k', 'orange'], ['solid', 'solid', 'solid'],
                             'Absolute Humidity',
                             y_scales.get('Absolute_Humidity') if y_scales else None)

        # Calculate Comfort Index
        bed_data['Comfort_Index'] = calculate_comfort_index(bed_data['Tempereture'], bed_data['Humidity'])
        out_data['Comfort_Index'] = calculate_comfort_index(out_data['Temperature-outside'],
                                                            out_data['Humidity-outside'])
        lr_data['Comfort_Index'] = calculate_comfort_index(lr_data['Temperature_DS18B20'], lr_data['Humidity_DHT11'])

        ax2 = create_subplot(2, 6, bed_data.index, {
            'BedRoom Comfort Index': bed_data['Comfort_Index'],
            'Outside Comfort Index': out_data['Comfort_Index'],
            'LR Comfort Index': lr_data['Comfort_Index']
        },
                             ['BedRoom', 'Outside', 'DiningRoom'], ['r', 'g', 'blue'], ['solid', 'dotted', 'dotted'],
                             'Comfort Index',
                             y_scales.get('Comfort_Index') if y_scales else None, sharex=ax1)

        # Calculate Pressure Changes
        bed_data['Pressure_Change'] = bed_data['Pressure'].diff().fillna(0)
        out_data['Pressure_Change'] = out_data['Pressure-outside'].diff().fillna(0)

        ax3 = create_subplot(3, 6, bed_data.index, {
            'BedRoom Pressure Change': bed_data['Pressure_Change'],
            'Outside Pressure Change': out_data['Pressure_Change']
        },
                             ['BedRoom', 'Outside'], ['b', 'm'], ['solid', 'dashed'], 'Pressure Change',
                             y_scales.get('Pressure_Change') if y_scales else None, sharex=ax1)

        # Calculate Gas Resistance Changes
        bed_data['GasResistance_Change'] = bed_data['GasResistance'].diff().fillna(0)
        out_data['GasResistance_Change'] = out_data['GasResistance-outside'].diff().fillna(0)

        ax4 = create_subplot(4, 6, bed_data.index, {
            'BedRoom GasResistance Change': bed_data['GasResistance_Change'],
            'Outside GasResistance Change': out_data['GasResistance_Change']
        },
                             ['BedRoom', 'Outside'], ['m', 'm'], ['solid', 'dashed'], 'GasResistance Change',
                             y_scales.get('GasResistance_Change') if y_scales else None, sharex=ax1)

        plt.xlim(x_start, x_end)
        plt.tight_layout()
        plt.draw()
        plt.pause(30)


plot_data(start_time="2024-08-21 22:30:00",
          y_scales={'Absolute_Humidity': (0, 30), 'Comfort_Index': (0, 30), 'Pressure_Change': (-1, 1),
                    'GasResistance_Change': (-10000, 10000)})
