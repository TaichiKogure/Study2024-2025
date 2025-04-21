import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import numpy as np
import os
from datetime import datetime

# Set the style for the plots
plt.style.use('default')

# Define the paths to the data files in the Trial2 folder
# Get the directory where this script is located
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

DATA_FILES = [
    (os.path.join(SCRIPT_DIR, "BedRoomEnv.csv"), 'current_time'),
    (os.path.join(SCRIPT_DIR, "OutsideEnv.csv"), 'current_time'),
    (os.path.join(SCRIPT_DIR, "Env_data_BME680.csv"), 'current_time'),
    (os.path.join(SCRIPT_DIR, "LR_env.csv"), 'current_time')
]

# Define paths for calculations and results
CALCULATIONS_FOLDER = os.path.join(SCRIPT_DIR, "calculations")
RESULTS_FOLDER = os.path.join(SCRIPT_DIR, "results")

# Ensure the folders exist
os.makedirs(CALCULATIONS_FOLDER, exist_ok=True)
os.makedirs(RESULTS_FOLDER, exist_ok=True)


def load_data(file_path, date_column):
    """
    Load data from a CSV file and set the specified column as datetime index.
    Handles missing data by converting empty strings to NaN.
    """
    try:
        data = pd.read_csv(file_path, na_values=['', 'None'])
        data[date_column] = pd.to_datetime(data[date_column])
        data.set_index(date_column, inplace=True)
        return data
    except Exception as e:
        print(f"Error loading {file_path}: {e}")
        return pd.DataFrame()


def resample_data(data, rule='min', reference_index=None):
    """
    Resample the data to a consistent frequency.
    If reference_index is provided, align data to this index.
    """
    if data.empty:
        return data

    resampled_data = data.resample(rule).mean()

    if reference_index is not None:
        resampled_data = resampled_data.reindex(reference_index, method='nearest')

    return resampled_data


def plot_lines(ax, x_data, y_data_dict, labels, colors, linestyles, ylabel, y_scale=None):
    """
    Plot multiple lines on a single subplot.
    Skip any data series that contains only NaN values.
    """
    for y_data, label, color, linestyle in zip(y_data_dict, labels, colors, linestyles):
        # Skip if the data is all NaN
        if not y_data_dict[y_data].isna().all():
            ax.plot(x_data, y_data_dict[y_data], label=label, color=color, linestyle=linestyle)

    ax.set_ylabel(ylabel)
    if y_scale:
        ax.set_ylim(y_scale)
    ax.grid(True)
    ax.legend(loc='best')


def create_subplot(index, row, x_data, y_data_dict, labels, colors, linestyles, ylabel, y_scale=None, sharex=None):
    """
    Create a subplot for the data visualization.
    """
    ax = plt.subplot(3, 2, index, sharex=sharex)
    plot_lines(ax, x_data, y_data_dict, labels, colors, linestyles, ylabel, y_scale)
    if index >= 5:
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%Y-%m-%d %H:%M:%S'))
        plt.xticks(rotation=45)
    return ax


def calculate_absolute_humidity(temp, rh):
    """
    Calculate absolute humidity from temperature and relative humidity.
    Returns NaN for any invalid inputs.
    """
    try:
        # Convert inputs to numpy arrays to handle NaN values properly
        temp_array = np.array(temp)
        rh_array = np.array(rh)

        # Calculate absolute humidity
        ah = (6.112 * np.exp((17.67 * temp_array) / (temp_array + 243.5)) * rh_array * 2.1674) / (273.15 + temp_array)

        return ah
    except Exception as e:
        print(f"Error calculating absolute humidity: {e}")
        return np.nan


def save_calculation(data, filename):
    """
    Save intermediate calculation results to the calculations folder.
    """
    try:
        filepath = os.path.join(CALCULATIONS_FOLDER, filename)
        data.to_csv(filepath)
        print(f"Calculation saved to {filepath}")
    except Exception as e:
        print(f"Error saving calculation: {e}")


def plot_data(start_time=None, end_time=None, y_scales=None, save_fig=True):
    """
    Plot environmental data with optional time filtering.

    Parameters:
    - start_time: Optional start time for filtering data (string in format 'YYYY-MM-DD HH:MM:SS')
    - end_time: Optional end time for filtering data (string in format 'YYYY-MM-DD HH:MM:SS')
    - y_scales: Dictionary of y-axis scales for each subplot
    - save_fig: Whether to save the figure to the results folder
    """
    plt.figure(figsize=(15, 10))

    # Read and preprocess data
    datasets = []
    for file_path, date_col in DATA_FILES:
        data = load_data(file_path, date_col)
        datasets.append(data)

    # Check if any datasets are empty
    if any(data.empty for data in datasets):
        print("Error: One or more datasets could not be loaded.")
        return

    # Use first dataset's index as reference
    reference_index = datasets[0].index

    # Resample all datasets
    resampled_datasets = [resample_data(data, reference_index=reference_index) for data in datasets]

    # Unpack datasets
    bed_data, out_data, bme680_data, lr_data = resampled_datasets

    # Filter data by time period if specified
    if start_time:
        start_time = pd.to_datetime(start_time)
        bed_data = bed_data[bed_data.index >= start_time]
        out_data = out_data[out_data.index >= start_time]
        bme680_data = bme680_data[bme680_data.index >= start_time]
        lr_data = lr_data[lr_data.index >= start_time]

    if end_time:
        end_time = pd.to_datetime(end_time)
        bed_data = bed_data[bed_data.index <= end_time]
        out_data = out_data[out_data.index <= end_time]
        bme680_data = bme680_data[bme680_data.index <= end_time]
        lr_data = lr_data[lr_data.index <= end_time]

    # Calculate absolute humidity
    try:
        bed_data['Absolute_Humidity'] = calculate_absolute_humidity(bed_data['Tempereture'], bed_data['Humidity'])
        out_data['Absolute_Humidity'] = calculate_absolute_humidity(out_data['Temperature-outside'], out_data['Humidity-outside'])
        lr_data['Absolute_Humidity'] = calculate_absolute_humidity(lr_data['Temperature_DS18B20'], lr_data['Humidity_DHT11'])

        # Save calculations
        save_calculation(bed_data[['Absolute_Humidity']], 'bedroom_absolute_humidity.csv')
        save_calculation(out_data[['Absolute_Humidity']], 'outside_absolute_humidity.csv')
        save_calculation(lr_data[['Absolute_Humidity']], 'livingroom_absolute_humidity.csv')
    except Exception as e:
        print(f"Error in absolute humidity calculation: {e}")

    # Set x-axis limits
    x_start = bed_data.index.min()
    x_end = bed_data.index.max()

    # Create subplots
    ax1 = create_subplot(1, 6, bed_data.index, {
        'CO2_Bedroom': bed_data['CO2'],
        'CO2_LR': lr_data['CO2']
    },
                         ['Bedroom', 'Living Room'], ['c', 'k'], ['solid', 'solid'], 'CO2 (ppm)',
                         y_scales.get('CO2') if y_scales else None)

    ax2 = create_subplot(2, 6, bed_data.index, {
        'BedRoom Temperature': bed_data['Tempereture'],
        'Outside Temperature': out_data['Temperature-outside'],
        'BME680 Temperature': bme680_data['temperature'],
        'LR Temperature': lr_data['Temperature_DS18B20']
    },
                         ['Bedroom', 'Outside', 'BME680', 'Living Room'],
                         ['r', 'r', 'orange', 'brown'],
                         ['solid', 'dashed', 'dotted', 'solid'],
                         'Temperature (°C)',
                         y_scales.get('Temperature') if y_scales else None, sharex=ax1)

    ax3 = create_subplot(3, 6, bed_data.index, {
        'BedRoom Humidity': bed_data['Humidity'],
        'Outside Humidity': out_data['Humidity-outside'],
        'LR Humidity': lr_data['Humidity_DHT11']
    },
                         ['Bedroom', 'Outside', 'Living Room'],
                         ['g', 'g', 'cyan'],
                         ['solid', 'dashed', 'dashdot'],
                         'Humidity (%)',
                         y_scales.get('Humidity') if y_scales else None, sharex=ax1)

    ax4 = create_subplot(4, 6, bed_data.index, {
        'BedRoom Pressure': bed_data['Pressure'],
        'Outside Pressure': out_data['Pressure-outside']
    },
                         ['Bedroom', 'Outside'],
                         ['b', 'b'],
                         ['solid', 'dashed'],
                         'Pressure (hPa)',
                         y_scales.get('Pressure') if y_scales else None, sharex=ax1)

    ax5 = create_subplot(5, 6, bed_data.index, {
        'Absolute Humidity - BedRoom': bed_data['Absolute_Humidity'],
        'Absolute Humidity - Outside': out_data['Absolute_Humidity'],
        'Absolute Humidity - LivingRoom': lr_data['Absolute_Humidity']
    },
                         ['Bedroom', 'Outside', 'Living Room'],
                         ['b', 'r', 'g'],
                         ['solid', 'dashed', '-.'],
                         'Absolute Humidity (g/m³)',
                         y_scales.get('Absolute Humidity') if y_scales else None, sharex=ax1)

    # Calculate and plot normalized GasResistance values
    try:
        bed_data['GasResistance_Avg'] = bed_data['GasResistance'].expanding().mean()
        out_data['GasResistance_Avg'] = out_data['GasResistance-outside'].expanding().mean()
        lr_data['AnalogValue_Avg'] = lr_data['AnalogValue'].expanding().mean()

        bed_data['GasResistance_Normalized'] = bed_data['GasResistance'] / bed_data['GasResistance_Avg'].shift(1).fillna(1)
        out_data['GasResistance_Normalized'] = out_data['GasResistance-outside'] / out_data['GasResistance_Avg'].shift(1).fillna(1)
        lr_data['AnalogValue_Normalized'] = lr_data['AnalogValue'] / lr_data['AnalogValue_Avg'].shift(1).fillna(1)

        # Save calculations
        save_calculation(bed_data[['GasResistance_Normalized']], 'bedroom_gas_normalized.csv')
        save_calculation(out_data[['GasResistance_Normalized']], 'outside_gas_normalized.csv')
        save_calculation(lr_data[['AnalogValue_Normalized']], 'livingroom_gas_normalized.csv')
    except Exception as e:
        print(f"Error in gas resistance normalization: {e}")

    ax6 = create_subplot(6, 6, bed_data.index, {
        'BedRoom GasResistance Normalized': bed_data.get('GasResistance_Normalized', pd.Series()),
        'Outside GasResistance Normalized': out_data.get('GasResistance_Normalized', pd.Series()),
        'LR Gas Normalized': lr_data.get('AnalogValue_Normalized', pd.Series())
    },
                         ['Bedroom', 'Outside', 'Living Room'],
                         ['m', 'red', 'gray'],
                         ['solid', 'dashed', 'solid'],
                         'Gas Resistance (Normalized)',
                         y_scales.get('GasResistance Normalized') if y_scales else None, sharex=ax1)

    plt.xlim(x_start, x_end)

    # Add title with time range
    time_range = f"{x_start.strftime('%Y-%m-%d %H:%M')} to {x_end.strftime('%Y-%m-%d %H:%M')}"
    plt.suptitle(f"Environmental Data Analysis - {time_range}", fontsize=16)

    plt.tight_layout(rect=[0, 0, 1, 0.97])  # Make room for the title

    # Save the figure if requested
    if save_fig:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(RESULTS_FOLDER, f"environmental_data_{timestamp}.png")
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"Figure saved to {filename}")

    plt.show()


def main():
    """
    Main function to run the environmental data analysis.
    """
    print("TRY2 Environmental Data Analysis")
    print("--------------------------------")
    print("Available options:")
    print("1. Analyze all data")
    print("2. Analyze data for a specific time period")

    choice = input("Enter your choice (1 or 2): ")

    if choice == '1':
        # Analyze all data
        plot_data(y_scales={
            'CO2': (400, 1500),
            'Temperature': (20, 40),
            'Humidity': (30, 90),
            'Pressure': (990, 1010),
            'GasResistance Normalized': (0.4, 1.7),
            'Absolute Humidity': (5, 30)
        })
    elif choice == '2':
        # Analyze data for a specific time period
        start_time = input("Enter start time (YYYY-MM-DD HH:MM:SS) or leave blank for earliest: ")
        end_time = input("Enter end time (YYYY-MM-DD HH:MM:SS) or leave blank for latest: ")

        # Use empty strings as None
        start_time = start_time if start_time else None
        end_time = end_time if end_time else None

        plot_data(
            start_time=start_time,
            end_time=end_time,
            y_scales={
                'CO2': (400, 1500),
                'Temperature': (20, 40),
                'Humidity': (30, 90),
                'Pressure': (990, 1010),
                'GasResistance Normalized': (0.4, 1.7),
                'Absolute Humidity': (5, 30)
            }
        )
    else:
        print("Invalid choice. Please run the program again.")


if __name__ == "__main__":
    main()
