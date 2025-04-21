# TRY2 Environmental Data Analysis

This program reads and graphs environmental data from CSV files in the Trial2 folder. It allows you to analyze data for any specified time period and handles missing data gracefully.

## Features

- Reads data from multiple CSV files (BedRoomEnv.csv, OutsideEnv.csv, Env_data_BME680.csv, LR_env.csv)
- Allows filtering data by specific time periods
- Handles missing data by skipping those parts without describing them
- Creates visualizations with multiple subplots for different environmental parameters
- Saves intermediate calculations to the `calculations` folder
- Saves visualization results to the `results` folder

## Usage

1. Make sure all data files are in the Trial2 folder
2. Run the script `TRY2_environmental_data_analysis.py`
3. Choose one of the available options:
   - Option 1: Analyze all data
   - Option 2: Analyze data for a specific time period (you'll be prompted to enter start and end times)

### Time Period Format

When specifying a time period, use the format: `YYYY-MM-DD HH:MM:SS`

Examples:
- `2024-08-15 00:00:00`
- `2024-09-01 12:30:00`

You can leave the input blank to use the earliest/latest available data.

## Output

The program generates:

1. **Visualizations**: Six subplots showing:
   - CO2 levels
   - Temperature readings
   - Humidity levels
   - Pressure readings
   - Absolute humidity (calculated)
   - Normalized gas resistance values

2. **Calculation Files**: Saved in the `calculations` folder:
   - Absolute humidity calculations
   - Normalized gas resistance values

3. **Result Images**: Saved in the `results` folder with timestamps in the filename

## Data Structure

The program expects the following data files with these column structures:

1. **BedRoomEnv.csv**:
   - current_time: Timestamp
   - CO2: CO2 levels
   - Tempereture: Temperature readings
   - Humidity: Humidity levels
   - Pressure: Pressure readings
   - GasResistance: Gas resistance values

2. **OutsideEnv.csv**:
   - current_time: Timestamp
   - Temperature-outside: Temperature readings
   - Humidity-outside: Humidity levels
   - Pressure-outside: Pressure readings
   - GasResistance-outside: Gas resistance values

3. **LR_env.csv**:
   - current_time: Timestamp
   - CO2: CO2 levels
   - AnalogValue: Analog sensor readings
   - Temperature_DS18B20: Temperature readings
   - Humidity_DHT11: Humidity levels

4. **Env_data_BME680.csv**:
   - current_time: Timestamp
   - Temperature: Temperature readings
   - Other environmental parameters

## Troubleshooting

If you encounter any issues:

1. Make sure all data files are in the correct location
2. Check that the data files have the expected column names
3. If there are errors with specific calculations, the program will continue and skip those parts