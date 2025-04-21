import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
import matplotlib.dates as mdates
from datetime import datetime
import os
from dataclasses import dataclass
from typing import Dict, List

@dataclass
class TimeConstants:
    SEASONS = {
        'Spring': range(4, 7),
        'Summer': range(7, 10),
        'Fall': range(10, 13),
        'Winter': range(1, 4)
    }
    
    TIME_OF_DAY = {
        'Morning': range(5, 12),
        'Afternoon': range(12, 17),
        'Evening': range(17, 22),
        'Night': list(range(22, 24)) + list(range(0, 5))
    }

class EnvironmentalDataAnalyzer:
    def __init__(self, data_files: Dict[str, str], output_dir: str):
        self.data_files = data_files
        self.output_dir = output_dir
        self.datasets = {}
        
        # プロットスタイルの設定
# Set style for plots
        plt.style.use('seaborn-v0_8-whitegrid')
        sns.set_palette("viridis")
        
        # 出力ディレクトリの作成
        os.makedirs(output_dir, exist_ok=True)

# Define seasons
    def get_season(self, month: int) -> str:
        return next(season for season, months in TimeConstants.SEASONS.items()
                   if month in months)

# Define time of day
    def get_time_of_day(self, hour: int) -> str:
        return next(period for period, hours in TimeConstants.TIME_OF_DAY.items()
                   if hour in hours)

# Calculate absolute humidity
    def calculate_absolute_humidity(self, temp: float, rh: float) -> float:
        return (6.112 * np.exp((17.67 * temp) / (temp + 243.5)) * rh * 2.1674) / (273.15 + temp)

# Load and preprocess data
    def load_data(self, file_path: str, date_column: str = 'current_time') -> pd.DataFrame:
        print(f"Loading data from {file_path}...")
        data = pd.read_csv(file_path)

    # Convert date column to datetime
        data[date_column] = pd.to_datetime(data[date_column])

    # Set date column as index
        data.set_index(date_column, inplace=True)
        
        # 時間関連の特徴量を追加
    # Add month, day, hour columns for analysis
        data['month'] = data.index.month
        data['day'] = data.index.day
        data['hour'] = data.index.hour
        data['weekday'] = data.index.weekday
        data['is_weekend'] = data['weekday'].apply(lambda x: 1 if x >= 5 else 0)

    # Add season and time of day
        data['season'] = data['month'].apply(self.get_season)
        data['time_of_day'] = data['hour'].apply(self.get_time_of_day)
        
        # 欠損値の処理
    # Handle missing values
        data = data.replace(0, np.nan)

        print(f"Data loaded. Shape: {data.shape}")
        return data

    def calculate_absolute_humidity_for_location(self, location: str, data: pd.DataFrame) -> pd.DataFrame:
        temp_humidity_mapping = {
            'bedroom': ('Tempereture', 'Humidity'),
            'outside': ('Temperature-outside', 'Humidity-outside'),
            'living_room': ('Temperature_DS18B20', 'Humidity_DHT11'),
            'desk': ('temperature', 'humidity')
        }
        
        if location in temp_humidity_mapping:
            temp_col, hum_col = temp_humidity_mapping[location]
            if temp_col in data.columns and hum_col in data.columns:
                data['Absolute_Humidity'] = self.calculate_absolute_humidity(
                    data[temp_col].fillna(0),
                    data[hum_col].fillna(0)
                )
        return data

    def load_all_datasets(self):
        for location, file_path in self.data_files.items():
            try:
                data = self.load_data(file_path)
                data = self.calculate_absolute_humidity_for_location(location, data)
                self.datasets[location] = data
            except Exception as e:
                print(f"Error loading {file_path}: {e}")
                
    def analyze(self):
        self.load_all_datasets()
        analysis_functions = [
            self.analyze_daily_trends,
            self.analyze_co2_trends,
            self.analyze_co2_humidity_relationship,
            # ... その他の分析関数
        ]
        
        for analyze_func in analysis_functions:
            try:
                analyze_func()
            except Exception as e:
                print(f"Error in {analyze_func.__name__}: {e}")

    # 以下、個別の分析メソッドは既存のコードと同様...
    analyze_weekday_weekend_patterns(datasets, output_dir)
    analyze_temperature_variations(datasets, output_dir)
    analyze_humidity_patterns(datasets, output_dir)
    analyze_extreme_values(datasets, output_dir)
    analyze_seasonal_trends(datasets, output_dir)
    analyze_pressure_trends(datasets, output_dir)
    analyze_monthly_trends(datasets, output_dir)

    # Machine learning analysis
    ml_analysis_temperature(datasets, output_dir)
    ml_analysis_co2(datasets, output_dir)

# Analysis functions
def analyze_daily_trends(datasets, output_dir):
    """Analyze and visualize daily trends for temperature, humidity, and CO2."""
    plt.figure(figsize=(15, 10))

    # Plot 1: Temperature by hour of day
    plt.subplot(2, 2, 1)
    for location, data in datasets.items():
        if location == 'bedroom' and 'Tempereture' in data.columns:
            hourly_temp = data.groupby('hour')['Tempereture'].mean()
            plt.plot(hourly_temp.index, hourly_temp.values, label='Bedroom')
        elif location == 'outside' and 'Temperature-outside' in data.columns:
            hourly_temp = data.groupby('hour')['Temperature-outside'].mean()
            plt.plot(hourly_temp.index, hourly_temp.values, label='Outside')
        elif location == 'living_room' and 'Temperature_DS18B20' in data.columns:
            hourly_temp = data.groupby('hour')['Temperature_DS18B20'].mean()
            plt.plot(hourly_temp.index, hourly_temp.values, label='Living Room')
        elif location == 'desk' and 'temperature' in data.columns:
            hourly_temp = data.groupby('hour')['temperature'].mean()
            plt.plot(hourly_temp.index, hourly_temp.values, label='Desk')

    plt.title('Average Temperature by Hour of Day')
    plt.xlabel('Hour of Day')
    plt.ylabel('Temperature (°C)')
    plt.xticks(range(0, 24, 2))
    plt.legend()
    plt.grid(True)

    # Plot 2: Humidity by hour of day
    plt.subplot(2, 2, 2)
    for location, data in datasets.items():
        if location == 'bedroom' and 'Humidity' in data.columns:
            hourly_hum = data.groupby('hour')['Humidity'].mean()
            plt.plot(hourly_hum.index, hourly_hum.values, label='Bedroom')
        elif location == 'outside' and 'Humidity-outside' in data.columns:
            hourly_hum = data.groupby('hour')['Humidity-outside'].mean()
            plt.plot(hourly_hum.index, hourly_hum.values, label='Outside')
        elif location == 'living_room' and 'Humidity_DHT11' in data.columns:
            hourly_hum = data.groupby('hour')['Humidity_DHT11'].mean()
            plt.plot(hourly_hum.index, hourly_hum.values, label='Living Room')
        elif location == 'desk' and 'humidity' in data.columns:
            hourly_hum = data.groupby('hour')['humidity'].mean()
            plt.plot(hourly_hum.index, hourly_hum.values, label='Desk')

    plt.title('Average Humidity by Hour of Day')
    plt.xlabel('Hour of Day')
    plt.ylabel('Humidity (%)')
    plt.xticks(range(0, 24, 2))
    plt.legend()
    plt.grid(True)

    # Plot 3: CO2 by hour of day
    plt.subplot(2, 2, 3)
    for location, data in datasets.items():
        if location == 'bedroom' and 'CO2' in data.columns:
            hourly_co2 = data.groupby('hour')['CO2'].mean()
            plt.plot(hourly_co2.index, hourly_co2.values, label='Bedroom')
        elif location == 'living_room' and 'CO2' in data.columns:
            hourly_co2 = data.groupby('hour')['CO2'].mean()
            plt.plot(hourly_co2.index, hourly_co2.values, label='Living Room')

    plt.title('Average CO2 by Hour of Day')
    plt.xlabel('Hour of Day')
    plt.ylabel('CO2 (ppm)')
    plt.xticks(range(0, 24, 2))
    plt.legend()
    plt.grid(True)

    # Plot 4: Absolute Humidity by hour of day
    plt.subplot(2, 2, 4)
    for location, data in datasets.items():
        if 'Absolute_Humidity' in data.columns:
            hourly_abs_hum = data.groupby('hour')['Absolute_Humidity'].mean()
            plt.plot(hourly_abs_hum.index, hourly_abs_hum.values, label=location.replace('_', ' ').title())

    plt.title('Average Absolute Humidity by Hour of Day')
    plt.xlabel('Hour of Day')
    plt.ylabel('Absolute Humidity (g/m³)')
    plt.xticks(range(0, 24, 2))
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/daily_trends.png', dpi=300)
    plt.close()

def analyze_co2_trends(datasets, output_dir):
    """Analyze and visualize CO2 trends by time of day and season."""
    plt.figure(figsize=(15, 10))

    # Plot 1: CO2 by hour of day for each location
    plt.subplot(2, 2, 1)
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            hourly_co2 = data.groupby('hour')['CO2'].mean()
            plt.plot(hourly_co2.index, hourly_co2.values, label=location.replace('_', ' ').title())

    plt.title('Average CO2 by Hour of Day')
    plt.xlabel('Hour of Day')
    plt.ylabel('CO2 (ppm)')
    plt.xticks(range(0, 24, 2))
    plt.legend()
    plt.grid(True)

    # Plot 2: CO2 by season for each location
    plt.subplot(2, 2, 2)
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            seasonal_co2 = data.groupby('season')['CO2'].mean()
            # Reorder seasons
            season_order = ['Winter', 'Spring', 'Summer', 'Fall']
            seasonal_co2 = seasonal_co2.reindex(season_order)
            plt.bar(
                [i + 0.2 * list(datasets.keys()).index(location) for i in range(len(seasonal_co2))], 
                seasonal_co2.values, 
                width=0.2, 
                label=location.replace('_', ' ').title()
            )

    plt.title('Average CO2 by Season')
    plt.xlabel('Season')
    plt.ylabel('CO2 (ppm)')
    plt.xticks(range(4), season_order)
    plt.legend()
    plt.grid(True)

    # Plot 3: CO2 by time of day for each location
    plt.subplot(2, 2, 3)
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            tod_co2 = data.groupby('time_of_day')['CO2'].mean()
            # Reorder time of day
            tod_order = ['Morning', 'Afternoon', 'Evening', 'Night']
            tod_co2 = tod_co2.reindex(tod_order)
            plt.bar(
                [i + 0.2 * list(datasets.keys()).index(location) for i in range(len(tod_co2))], 
                tod_co2.values, 
                width=0.2, 
                label=location.replace('_', ' ').title()
            )

    plt.title('Average CO2 by Time of Day')
    plt.xlabel('Time of Day')
    plt.ylabel('CO2 (ppm)')
    plt.xticks(range(4), tod_order)
    plt.legend()
    plt.grid(True)

    # Plot 4: CO2 by weekday/weekend for each location
    plt.subplot(2, 2, 4)
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            weekend_co2 = data.groupby('is_weekend')['CO2'].mean()
            plt.bar(
                [i + 0.2 * list(datasets.keys()).index(location) for i in range(len(weekend_co2))], 
                weekend_co2.values, 
                width=0.2, 
                label=location.replace('_', ' ').title()
            )

    plt.title('Average CO2 by Weekday/Weekend')
    plt.xlabel('Day Type')
    plt.ylabel('CO2 (ppm)')
    plt.xticks([0, 1], ['Weekday', 'Weekend'])
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/co2_trends.png', dpi=300)
    plt.close()

def analyze_co2_humidity_relationship(datasets, output_dir):
    """Analyze and visualize the relationship between CO2 and humidity."""
    # Time series plot
    plt.figure(figsize=(15, 10))

    # Plot 1: CO2 and Humidity time series for Bedroom
    if 'bedroom' in datasets and 'CO2' in datasets['bedroom'].columns and 'Humidity' in datasets['bedroom'].columns:
        ax1 = plt.subplot(2, 1, 1)
        data = datasets['bedroom'].resample('D').mean(numeric_only=True)  # Resample to daily averages

        ax1.plot(data.index, data['CO2'], 'b-', label='CO2')
        ax1.set_ylabel('CO2 (ppm)', color='b')
        ax1.tick_params('y', colors='b')

        ax2 = ax1.twinx()
        ax2.plot(data.index, data['Humidity'], 'r-', label='Humidity')
        ax2.set_ylabel('Humidity (%)', color='r')
        ax2.tick_params('y', colors='r')

        plt.title('Bedroom CO2 and Humidity Time Series')
        plt.grid(True)

        # Add legend
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left')

    # Plot 2: CO2 and Humidity time series for Living Room
    if 'living_room' in datasets and 'CO2' in datasets['living_room'].columns and 'Humidity_DHT11' in datasets['living_room'].columns:
        ax3 = plt.subplot(2, 1, 2)
        data = datasets['living_room'].resample('D').mean(numeric_only=True)  # Resample to daily averages

        ax3.plot(data.index, data['CO2'], 'b-', label='CO2')
        ax3.set_ylabel('CO2 (ppm)', color='b')
        ax3.tick_params('y', colors='b')

        ax4 = ax3.twinx()
        ax4.plot(data.index, data['Humidity_DHT11'], 'r-', label='Humidity')
        ax4.set_ylabel('Humidity (%)', color='r')
        ax4.tick_params('y', colors='r')

        plt.title('Living Room CO2 and Humidity Time Series')
        plt.grid(True)

        # Add legend
        lines3, labels3 = ax3.get_legend_handles_labels()
        lines4, labels4 = ax4.get_legend_handles_labels()
        ax3.legend(lines3 + lines4, labels3 + labels4, loc='upper left')

    plt.tight_layout()
    plt.savefig(f'{output_dir}/co2_humidity_time_series.png', dpi=300)
    plt.close()

    # Scatter plot
    plt.figure(figsize=(15, 10))

    # Plot 1: CO2 vs Humidity scatter plot for Bedroom
    plt.subplot(2, 2, 1)
    if 'bedroom' in datasets and 'CO2' in datasets['bedroom'].columns and 'Humidity' in datasets['bedroom'].columns:
        data = datasets['bedroom'].dropna(subset=['CO2', 'Humidity'])
        plt.scatter(data['Humidity'], data['CO2'], alpha=0.5)

        # Add trend line
        z = np.polyfit(data['Humidity'], data['CO2'], 1)
        p = np.poly1d(z)
        plt.plot(data['Humidity'], p(data['Humidity']), "r--")

        plt.title('Bedroom CO2 vs Humidity')
        plt.xlabel('Humidity (%)')
        plt.ylabel('CO2 (ppm)')
        plt.grid(True)

    # Plot 2: CO2 vs Humidity scatter plot for Living Room
    plt.subplot(2, 2, 2)
    if 'living_room' in datasets and 'CO2' in datasets['living_room'].columns and 'Humidity_DHT11' in datasets['living_room'].columns:
        data = datasets['living_room'].dropna(subset=['CO2', 'Humidity_DHT11'])
        plt.scatter(data['Humidity_DHT11'], data['CO2'], alpha=0.5)

        # Add trend line
        z = np.polyfit(data['Humidity_DHT11'], data['CO2'], 1)
        p = np.poly1d(z)
        plt.plot(data['Humidity_DHT11'], p(data['Humidity_DHT11']), "r--")

        plt.title('Living Room CO2 vs Humidity')
        plt.xlabel('Humidity (%)')
        plt.ylabel('CO2 (ppm)')
        plt.grid(True)

    # Plot 3: CO2 vs Absolute Humidity scatter plot for Bedroom
    plt.subplot(2, 2, 3)
    if 'bedroom' in datasets and 'CO2' in datasets['bedroom'].columns and 'Absolute_Humidity' in datasets['bedroom'].columns:
        data = datasets['bedroom'].dropna(subset=['CO2', 'Absolute_Humidity'])
        plt.scatter(data['Absolute_Humidity'], data['CO2'], alpha=0.5)

        # Add trend line
        z = np.polyfit(data['Absolute_Humidity'], data['CO2'], 1)
        p = np.poly1d(z)
        plt.plot(data['Absolute_Humidity'], p(data['Absolute_Humidity']), "r--")

        plt.title('Bedroom CO2 vs Absolute Humidity')
        plt.xlabel('Absolute Humidity (g/m³)')
        plt.ylabel('CO2 (ppm)')
        plt.grid(True)

    # Plot 4: CO2 vs Absolute Humidity scatter plot for Living Room
    plt.subplot(2, 2, 4)
    if 'living_room' in datasets and 'CO2' in datasets['living_room'].columns and 'Absolute_Humidity' in datasets['living_room'].columns:
        data = datasets['living_room'].dropna(subset=['CO2', 'Absolute_Humidity'])
        plt.scatter(data['Absolute_Humidity'], data['CO2'], alpha=0.5)

        # Add trend line
        z = np.polyfit(data['Absolute_Humidity'], data['CO2'], 1)
        p = np.poly1d(z)
        plt.plot(data['Absolute_Humidity'], p(data['Absolute_Humidity']), "r--")

        plt.title('Living Room CO2 vs Absolute Humidity')
        plt.xlabel('Absolute Humidity (g/m³)')
        plt.ylabel('CO2 (ppm)')
        plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/co2_humidity_relationship.png', dpi=300)
    plt.close()

def analyze_weekday_weekend_patterns(datasets, output_dir):
    """Analyze and visualize weekday vs weekend patterns."""
    plt.figure(figsize=(15, 15))

    # Plot 1: Temperature by hour - Weekday vs Weekend
    plt.subplot(3, 2, 1)
    for location, data in datasets.items():
        temp_col = None
        if location == 'bedroom' and 'Tempereture' in data.columns:
            temp_col = 'Tempereture'
        elif location == 'outside' and 'Temperature-outside' in data.columns:
            temp_col = 'Temperature-outside'
        elif location == 'living_room' and 'Temperature_DS18B20' in data.columns:
            temp_col = 'Temperature_DS18B20'
        elif location == 'desk' and 'temperature' in data.columns:
            temp_col = 'temperature'

        if temp_col:
            # Weekday
            weekday_temp = data[data['is_weekend'] == 0].groupby('hour')[temp_col].mean()
            plt.plot(weekday_temp.index, weekday_temp.values, 
                     label=f"{location.replace('_', ' ').title()} - Weekday")

            # Weekend
            weekend_temp = data[data['is_weekend'] == 1].groupby('hour')[temp_col].mean()
            plt.plot(weekend_temp.index, weekend_temp.values, '--', 
                     label=f"{location.replace('_', ' ').title()} - Weekend")

    plt.title('Temperature by Hour - Weekday vs Weekend')
    plt.xlabel('Hour of Day')
    plt.ylabel('Temperature (°C)')
    plt.xticks(range(0, 24, 2))
    plt.legend()
    plt.grid(True)

    # Plot 2: Humidity by hour - Weekday vs Weekend
    plt.subplot(3, 2, 2)
    for location, data in datasets.items():
        hum_col = None
        if location == 'bedroom' and 'Humidity' in data.columns:
            hum_col = 'Humidity'
        elif location == 'outside' and 'Humidity-outside' in data.columns:
            hum_col = 'Humidity-outside'
        elif location == 'living_room' and 'Humidity_DHT11' in data.columns:
            hum_col = 'Humidity_DHT11'
        elif location == 'desk' and 'humidity' in data.columns:
            hum_col = 'humidity'

        if hum_col:
            # Weekday
            weekday_hum = data[data['is_weekend'] == 0].groupby('hour')[hum_col].mean()
            plt.plot(weekday_hum.index, weekday_hum.values, 
                     label=f"{location.replace('_', ' ').title()} - Weekday")

            # Weekend
            weekend_hum = data[data['is_weekend'] == 1].groupby('hour')[hum_col].mean()
            plt.plot(weekend_hum.index, weekend_hum.values, '--', 
                     label=f"{location.replace('_', ' ').title()} - Weekend")

    plt.title('Humidity by Hour - Weekday vs Weekend')
    plt.xlabel('Hour of Day')
    plt.ylabel('Humidity (%)')
    plt.xticks(range(0, 24, 2))
    plt.legend()
    plt.grid(True)

    # Plot 3: CO2 by hour - Weekday vs Weekend
    plt.subplot(3, 2, 3)
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            # Weekday
            weekday_co2 = data[data['is_weekend'] == 0].groupby('hour')['CO2'].mean()
            plt.plot(weekday_co2.index, weekday_co2.values, 
                     label=f"{location.replace('_', ' ').title()} - Weekday")

            # Weekend
            weekend_co2 = data[data['is_weekend'] == 1].groupby('hour')['CO2'].mean()
            plt.plot(weekend_co2.index, weekend_co2.values, '--', 
                     label=f"{location.replace('_', ' ').title()} - Weekend")

    plt.title('CO2 by Hour - Weekday vs Weekend')
    plt.xlabel('Hour of Day')
    plt.ylabel('CO2 (ppm)')
    plt.xticks(range(0, 24, 2))
    plt.legend()
    plt.grid(True)

    # Plot 4: Pressure by hour - Weekday vs Weekend
    plt.subplot(3, 2, 4)
    for location, data in datasets.items():
        press_col = None
        if location == 'bedroom' and 'Pressure' in data.columns:
            press_col = 'Pressure'
        elif location == 'outside' and 'Pressure-outside' in data.columns:
            press_col = 'Pressure-outside'
        elif location == 'desk' and 'pressure' in data.columns:
            press_col = 'pressure'

        if press_col:
            # Weekday
            weekday_press = data[data['is_weekend'] == 0].groupby('hour')[press_col].mean()
            plt.plot(weekday_press.index, weekday_press.values, 
                     label=f"{location.replace('_', ' ').title()} - Weekday")

            # Weekend
            weekend_press = data[data['is_weekend'] == 1].groupby('hour')[press_col].mean()
            plt.plot(weekend_press.index, weekend_press.values, '--', 
                     label=f"{location.replace('_', ' ').title()} - Weekend")

    plt.title('Pressure by Hour - Weekday vs Weekend')
    plt.xlabel('Hour of Day')
    plt.ylabel('Pressure (hPa)')
    plt.xticks(range(0, 24, 2))
    plt.legend()
    plt.grid(True)

    # Plot 5: Absolute Humidity by hour - Weekday vs Weekend
    plt.subplot(3, 2, 5)
    for location, data in datasets.items():
        if 'Absolute_Humidity' in data.columns:
            # Weekday
            weekday_abs_hum = data[data['is_weekend'] == 0].groupby('hour')['Absolute_Humidity'].mean()
            plt.plot(weekday_abs_hum.index, weekday_abs_hum.values, 
                     label=f"{location.replace('_', ' ').title()} - Weekday")

            # Weekend
            weekend_abs_hum = data[data['is_weekend'] == 1].groupby('hour')['Absolute_Humidity'].mean()
            plt.plot(weekend_abs_hum.index, weekend_abs_hum.values, '--', 
                     label=f"{location.replace('_', ' ').title()} - Weekend")

    plt.title('Absolute Humidity by Hour - Weekday vs Weekend')
    plt.xlabel('Hour of Day')
    plt.ylabel('Absolute Humidity (g/m³)')
    plt.xticks(range(0, 24, 2))
    plt.legend()
    plt.grid(True)

    # Plot 6: Gas Resistance by hour - Weekday vs Weekend
    plt.subplot(3, 2, 6)
    for location, data in datasets.items():
        gas_col = None
        if location == 'bedroom' and 'GasResistance' in data.columns:
            gas_col = 'GasResistance'
        elif location == 'outside' and 'GasResistance-outside' in data.columns:
            gas_col = 'GasResistance-outside'
        elif location == 'living_room' and 'AnalogValue' in data.columns:
            gas_col = 'AnalogValue'
        elif location == 'desk' and 'gas_resistance' in data.columns:
            gas_col = 'gas_resistance'

        if gas_col:
            # Weekday
            weekday_gas = data[data['is_weekend'] == 0].groupby('hour')[gas_col].mean()
            plt.plot(weekday_gas.index, weekday_gas.values, 
                     label=f"{location.replace('_', ' ').title()} - Weekday")

            # Weekend
            weekend_gas = data[data['is_weekend'] == 1].groupby('hour')[gas_col].mean()
            plt.plot(weekend_gas.index, weekend_gas.values, '--', 
                     label=f"{location.replace('_', ' ').title()} - Weekend")

    plt.title('Gas Resistance by Hour - Weekday vs Weekend')
    plt.xlabel('Hour of Day')
    plt.ylabel('Gas Resistance')
    plt.xticks(range(0, 24, 2))
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/weekday_weekend_patterns.png', dpi=300)
    plt.close()

def analyze_temperature_variations(datasets, output_dir):
    """Analyze and visualize temperature variations by season and time of day."""
    # ... (前のコードは同じ)
    plt.figure(figsize=(15, 10))

    # Plot 1: Temperature by season for each location
    plt.subplot(2, 2, 1)
    for location, data in datasets.items():
        temp_col = None
        if location == 'bedroom' and 'Tempereture' in data.columns:
            temp_col = 'Tempereture'
        elif location == 'outside' and 'Temperature-outside' in data.columns:
            temp_col = 'Temperature-outside'
        elif location == 'living_room' and 'Temperature_DS18B20' in data.columns:
            temp_col = 'Temperature_DS18B20'
        elif location == 'desk' and 'temperature' in data.columns:
            temp_col = 'temperature'

        if temp_col:
            seasonal_temp = data.groupby('season')[temp_col].mean()
            # Reorder seasons
            season_order = ['Winter', 'Spring', 'Summer', 'Fall']
            seasonal_temp = seasonal_temp.reindex(season_order)
            plt.bar(
                [i + 0.2 * list(datasets.keys()).index(location) for i in range(len(seasonal_temp))], 
                seasonal_temp.values, 
                width=0.2, 
                label=location.replace('_', ' ').title()
            )

    plt.title('Average Temperature by Season')
    plt.xlabel('Season')
    plt.ylabel('Temperature (°C)')
    plt.xticks(range(4), season_order)
    plt.legend()
    plt.grid(True)

    # Plot 2: Temperature by time of day for each location
    plt.subplot(2, 2, 2)
    for location, data in datasets.items():
        temp_col = None
        if location == 'bedroom' and 'Tempereture' in data.columns:
            temp_col = 'Tempereture'
        elif location == 'outside' and 'Temperature-outside' in data.columns:
            temp_col = 'Temperature-outside'
        elif location == 'living_room' and 'Temperature_DS18B20' in data.columns:
            temp_col = 'Temperature_DS18B20'
        elif location == 'desk' and 'temperature' in data.columns:
            temp_col = 'temperature'

        if temp_col:
            tod_temp = data.groupby('time_of_day')[temp_col].mean()
            # Reorder time of day
            tod_order = ['Morning', 'Afternoon', 'Evening', 'Night']
            tod_temp = tod_temp.reindex(tod_order)
            plt.bar(
                [i + 0.2 * list(datasets.keys()).index(location) for i in range(len(tod_temp))], 
                tod_temp.values, 
                width=0.2, 
                label=location.replace('_', ' ').title()
            )

    plt.title('Average Temperature by Time of Day')
    plt.xlabel('Time of Day')
    plt.ylabel('Temperature (°C)')
    plt.xticks(range(4), tod_order)
    plt.legend()
    plt.grid(True)

    # Plot 3: Temperature distribution by season
    plt.subplot(2, 2, 3)
    for location, data in datasets.items():
        temp_col = None
        if location == 'bedroom' and 'Tempereture' in data.columns:
            temp_col = 'Tempereture'
        elif location == 'outside' and 'Temperature-outside' in data.columns:
            temp_col = 'Temperature-outside'
        elif location == 'living_room' and 'Temperature_DS18B20' in data.columns:
            temp_col = 'Temperature_DS18B20'
        elif location == 'desk' and 'temperature' in data.columns:
            temp_col = 'temperature'

        if temp_col:
            # Create box plot for each season
            seasons = ['Winter', 'Spring', 'Summer', 'Fall']
            box_data = []
            for season in seasons:
                season_data = data[data['season'] == season][temp_col].dropna()
                if not season_data.empty:
                    box_data.append(season_data)
                else:
                    box_data.append([])

            # Create positions for box plots
            positions = [i + 0.2 * list(datasets.keys()).index(location) for i in range(len(seasons))]

            # Create box plot
            bp = plt.boxplot(box_data, positions=positions, widths=0.15, 
                            patch_artist=True, showfliers=False)

            # Set box colors
            for box in bp['boxes']:
                box.set(color='black', linewidth=1)
                box.set(facecolor=f'C{list(datasets.keys()).index(location)}')

    plt.title('Temperature Distribution by Season')
    plt.xlabel('Season')
    plt.ylabel('Temperature (°C)')
    plt.xticks(range(4), seasons)
    plt.legend([plt.Rectangle((0,0),1,1, facecolor=f'C{i}') 
               for i in range(len(datasets))], 
              [loc.replace('_', ' ').title() for loc in datasets.keys()])
    plt.grid(True)

    # Plot 4: Temperature range by month
    plt.subplot(2, 2, 4)
    for location, data in datasets.items():
        temp_col = None
        if location == 'bedroom' and 'Tempereture' in data.columns:
            temp_col = 'Tempereture'
        elif location == 'outside' and 'Temperature-outside' in data.columns:
            temp_col = 'Temperature-outside'
        elif location == 'living_room' and 'Temperature_DS18B20' in data.columns:
            temp_col = 'Temperature_DS18B20'
        elif location == 'desk' and 'temperature' in data.columns:
            temp_col = 'temperature'

        if temp_col:
            # 月別データを計算
            monthly_stats = data.groupby('month')[temp_col].agg(['min', 'max', 'mean'])
            months = monthly_stats.index

            # 実際のデータがある月のみをプロット
            plt.plot(months, monthly_stats['mean'], 'o-', 
                    label=f"{location.replace('_', ' ').title()} - Mean")
            plt.fill_between(months, monthly_stats['min'], 
                           monthly_stats['max'], alpha=0.2)

    plt.title('Temperature Range by Month')
    plt.xlabel('Month')
    plt.ylabel('Temperature (°C)')
    plt.xticks(range(1, 13))
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/temperature_variations.png', dpi=300)
    plt.close()

def analyze_humidity_patterns(datasets, output_dir):
    """Analyze and visualize humidity patterns by season and time of day."""
    plt.figure(figsize=(15, 10))

    # 前半のコードは変更なし...

    # Plot 1: Humidity by season for each location
    plt.subplot(2, 2, 1)
    for location, data in datasets.items():
        hum_col = None
        if location == 'bedroom' and 'Humidity' in data.columns:
            hum_col = 'Humidity'
        elif location == 'outside' and 'Humidity-outside' in data.columns:
            hum_col = 'Humidity-outside'
        elif location == 'living_room' and 'Humidity_DHT11' in data.columns:
            hum_col = 'Humidity_DHT11'
        elif location == 'desk' and 'humidity' in data.columns:
            hum_col = 'humidity'

        if hum_col:
            seasonal_hum = data.groupby('season')[hum_col].mean()
            # Reorder seasons
            season_order = ['Winter', 'Spring', 'Summer', 'Fall']
            seasonal_hum = seasonal_hum.reindex(season_order)
            plt.bar(
                [i + 0.2 * list(datasets.keys()).index(location) for i in range(len(seasonal_hum))], 
                seasonal_hum.values, 
                width=0.2, 
                label=location.replace('_', ' ').title()
            )

    plt.title('Average Humidity by Season')
    plt.xlabel('Season')
    plt.ylabel('Humidity (%)')
    plt.xticks(range(4), season_order)
    plt.legend()
    plt.grid(True)

    # Plot 2: Absolute Humidity by season for each location
    plt.subplot(2, 2, 2)
    for location, data in datasets.items():
        if 'Absolute_Humidity' in data.columns:
            seasonal_abs_hum = data.groupby('season')['Absolute_Humidity'].mean()
            # Reorder seasons
            season_order = ['Winter', 'Spring', 'Summer', 'Fall']
            seasonal_abs_hum = seasonal_abs_hum.reindex(season_order)
            plt.bar(
                [i + 0.2 * list(datasets.keys()).index(location) for i in range(len(seasonal_abs_hum))], 
                seasonal_abs_hum.values, 
                width=0.2, 
                label=location.replace('_', ' ').title()
            )

    plt.title('Average Absolute Humidity by Season')
    plt.xlabel('Season')
    plt.ylabel('Absolute Humidity (g/m³)')
    plt.xticks(range(4), season_order)
    plt.legend()
    plt.grid(True)

    # Plot 3: Humidity by time of day for each location
    plt.subplot(2, 2, 3)
    for location, data in datasets.items():
        hum_col = None
        if location == 'bedroom' and 'Humidity' in data.columns:
            hum_col = 'Humidity'
        elif location == 'outside' and 'Humidity-outside' in data.columns:
            hum_col = 'Humidity-outside'
        elif location == 'living_room' and 'Humidity_DHT11' in data.columns:
            hum_col = 'Humidity_DHT11'
        elif location == 'desk' and 'humidity' in data.columns:
            hum_col = 'humidity'

        if hum_col:
            tod_hum = data.groupby('time_of_day')[hum_col].mean()
            # Reorder time of day
            tod_order = ['Morning', 'Afternoon', 'Evening', 'Night']
            tod_hum = tod_hum.reindex(tod_order)
            plt.bar(
                [i + 0.2 * list(datasets.keys()).index(location) for i in range(len(tod_hum))], 
                tod_hum.values, 
                width=0.2, 
                label=location.replace('_', ' ').title()
            )

    plt.title('Average Humidity by Time of Day')
    plt.xlabel('Time of Day')
    plt.ylabel('Humidity (%)')
    plt.xticks(range(4), tod_order)
    plt.legend()
    plt.grid(True)

    # Plot 4: Humidity range by month（修正版）
    plt.subplot(2, 2, 4)
    for location, data in datasets.items():
        hum_col = None
        if location == 'bedroom' and 'Humidity' in data.columns:
            hum_col = 'Humidity'
        elif location == 'outside' and 'Humidity-outside' in data.columns:
            hum_col = 'Humidity-outside'
        elif location == 'living_room' and 'Humidity_DHT11' in data.columns:
            hum_col = 'Humidity_DHT11'
        elif location == 'desk' and 'humidity' in data.columns:
            hum_col = 'humidity'

        if hum_col:
            # 月別統計の計算
            monthly_stats = data.groupby('month')[hum_col].agg(['min', 'max', 'mean'])

            # 存在する月のデータのみをプロット
            available_months = monthly_stats.index.values

            plt.plot(available_months, monthly_stats['mean'], 'o-', 
                    label=f"{location.replace('_', ' ').title()} - Mean")
            plt.fill_between(available_months, 
                           monthly_stats['min'], 
                           monthly_stats['max'], 
                           alpha=0.2)

    plt.title('Humidity Range by Month')
    plt.xlabel('Month')
    plt.ylabel('Humidity (%)')
    plt.xticks(range(1, 13))
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/humidity_patterns.png', dpi=300)
    plt.close()

def analyze_extreme_values(datasets, output_dir):
    """Analyze and visualize extreme values for temperature, humidity, and CO2."""
    plt.figure(figsize=(15, 10))

    # Plot 1: Temperature extremes by location
    plt.subplot(2, 2, 1)
    for location, data in datasets.items():
        temp_col = None
        if location == 'bedroom' and 'Tempereture' in data.columns:
            temp_col = 'Tempereture'
        elif location == 'outside' and 'Temperature-outside' in data.columns:
            temp_col = 'Temperature-outside'
        elif location == 'living_room' and 'Temperature_DS18B20' in data.columns:
            temp_col = 'Temperature_DS18B20'
        elif location == 'desk' and 'temperature' in data.columns:
            temp_col = 'temperature'

        if temp_col:
            # Calculate percentiles
            p05 = data[temp_col].quantile(0.05)
            p25 = data[temp_col].quantile(0.25)
            p50 = data[temp_col].quantile(0.50)
            p75 = data[temp_col].quantile(0.75)
            p95 = data[temp_col].quantile(0.95)

            # Plot box plot
            box_pos = list(datasets.keys()).index(location)
            bp = plt.boxplot([data[temp_col].dropna()], positions=[box_pos], widths=0.6, 
                            patch_artist=True, showfliers=False)

            # Set box colors
            for box in bp['boxes']:
                box.set(color='black', linewidth=1)
                box.set(facecolor=f'C{box_pos}')

            # Add text for percentiles
            plt.text(box_pos, p95, f'{p95:.1f}°C', ha='center', va='bottom')
            plt.text(box_pos, p05, f'{p05:.1f}°C', ha='center', va='top')

    plt.title('Temperature Extremes by Location')
    plt.xlabel('Location')
    plt.ylabel('Temperature (°C)')
    plt.xticks(range(len(datasets)), [loc.replace('_', ' ').title() for loc in datasets.keys()])
    plt.grid(True)

    # Plot 2: Humidity extremes by location
    plt.subplot(2, 2, 2)
    for location, data in datasets.items():
        hum_col = None
        if location == 'bedroom' and 'Humidity' in data.columns:
            hum_col = 'Humidity'
        elif location == 'outside' and 'Humidity-outside' in data.columns:
            hum_col = 'Humidity-outside'
        elif location == 'living_room' and 'Humidity_DHT11' in data.columns:
            hum_col = 'Humidity_DHT11'
        elif location == 'desk' and 'humidity' in data.columns:
            hum_col = 'humidity'

        if hum_col:
            # Calculate percentiles
            p05 = data[hum_col].quantile(0.05)
            p25 = data[hum_col].quantile(0.25)
            p50 = data[hum_col].quantile(0.50)
            p75 = data[hum_col].quantile(0.75)
            p95 = data[hum_col].quantile(0.95)

            # Plot box plot
            box_pos = list(datasets.keys()).index(location)
            bp = plt.boxplot([data[hum_col].dropna()], positions=[box_pos], widths=0.6, 
                            patch_artist=True, showfliers=False)

            # Set box colors
            for box in bp['boxes']:
                box.set(color='black', linewidth=1)
                box.set(facecolor=f'C{box_pos}')

            # Add text for percentiles
            plt.text(box_pos, p95, f'{p95:.1f}%', ha='center', va='bottom')
            plt.text(box_pos, p05, f'{p05:.1f}%', ha='center', va='top')

    plt.title('Humidity Extremes by Location')
    plt.xlabel('Location')
    plt.ylabel('Humidity (%)')
    plt.xticks(range(len(datasets)), [loc.replace('_', ' ').title() for loc in datasets.keys()])
    plt.grid(True)

    # Plot 3: CO2 extremes by location
    plt.subplot(2, 2, 3)
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            # Calculate percentiles
            p05 = data['CO2'].quantile(0.05)
            p25 = data['CO2'].quantile(0.25)
            p50 = data['CO2'].quantile(0.50)
            p75 = data['CO2'].quantile(0.75)
            p95 = data['CO2'].quantile(0.95)

            # Plot box plot
            box_pos = list(datasets.keys()).index(location)
            bp = plt.boxplot([data['CO2'].dropna()], positions=[box_pos], widths=0.6, 
                            patch_artist=True, showfliers=False)

            # Set box colors
            for box in bp['boxes']:
                box.set(color='black', linewidth=1)
                box.set(facecolor=f'C{box_pos}')

            # Add text for percentiles
            plt.text(box_pos, p95, f'{p95:.0f}ppm', ha='center', va='bottom')
            plt.text(box_pos, p05, f'{p05:.0f}ppm', ha='center', va='top')

    plt.title('CO2 Extremes by Location')
    plt.xlabel('Location')
    plt.ylabel('CO2 (ppm)')
    plt.xticks(range(len(datasets)), [loc.replace('_', ' ').title() for loc in datasets.keys()])
    plt.grid(True)

    # Plot 4: Pressure extremes by location
    plt.subplot(2, 2, 4)
    for location, data in datasets.items():
        press_col = None
        if location == 'bedroom' and 'Pressure' in data.columns:
            press_col = 'Pressure'
        elif location == 'outside' and 'Pressure-outside' in data.columns:
            press_col = 'Pressure-outside'
        elif location == 'desk' and 'pressure' in data.columns:
            press_col = 'pressure'

        if press_col:
            # Calculate percentiles
            p05 = data[press_col].quantile(0.05)
            p25 = data[press_col].quantile(0.25)
            p50 = data[press_col].quantile(0.50)
            p75 = data[press_col].quantile(0.75)
            p95 = data[press_col].quantile(0.95)

            # Plot box plot
            box_pos = list(datasets.keys()).index(location)
            bp = plt.boxplot([data[press_col].dropna()], positions=[box_pos], widths=0.6, 
                            patch_artist=True, showfliers=False)

            # Set box colors
            for box in bp['boxes']:
                box.set(color='black', linewidth=1)
                box.set(facecolor=f'C{box_pos}')

            # Add text for percentiles
            plt.text(box_pos, p95, f'{p95:.1f}hPa', ha='center', va='bottom')
            plt.text(box_pos, p05, f'{p05:.1f}hPa', ha='center', va='top')

    plt.title('Pressure Extremes by Location')
    plt.xlabel('Location')
    plt.ylabel('Pressure (hPa)')
    plt.xticks(range(len(datasets)), [loc.replace('_', ' ').title() for loc in datasets.keys()])
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/extreme_values.png', dpi=300)
    plt.close()

def analyze_seasonal_trends(datasets, output_dir):
    plt.figure(figsize=(15, 15))

    # データフレームのインデックスが日時型であることを確認

    # Plot 1: Temperature by season and time of day
    plt.subplot(3, 2, 1)
    for location, data in datasets.items():
        temp_col = None
        if location == 'bedroom' and 'Tempereture' in data.columns:
            temp_col = 'Tempereture'
        elif location == 'outside' and 'Temperature-outside' in data.columns:
            temp_col = 'Temperature-outside'
        elif location == 'living_room' and 'Temperature_DS18B20' in data.columns:
            temp_col = 'Temperature_DS18B20'
        elif location == 'desk' and 'temperature' in data.columns:
            temp_col = 'temperature'

        if temp_col:
            # Group by season and time of day
            season_tod_temp = data.groupby(['season', 'time_of_day'])[temp_col].mean().unstack()

            # Reorder seasons and time of day
            season_order = ['Winter', 'Spring', 'Summer', 'Fall']
            tod_order = ['Morning', 'Afternoon', 'Evening', 'Night']
            season_tod_temp = season_tod_temp.reindex(season_order)[tod_order]

            # Plot heatmap
            plt.imshow(season_tod_temp.values, cmap='viridis', aspect='auto')
            plt.colorbar(label='Temperature (°C)')
            plt.xticks(range(len(tod_order)), tod_order)
            plt.yticks(range(len(season_order)), season_order)
            plt.title(f'{location.replace("_", " ").title()} Temperature by Season and Time of Day')
            break  # Only plot for the first location with temperature data

    # Plot 2: Humidity by season and time of day
    plt.subplot(3, 2, 2)
    for location, data in datasets.items():
        hum_col = None
        if location == 'bedroom' and 'Humidity' in data.columns:
            hum_col = 'Humidity'
        elif location == 'outside' and 'Humidity-outside' in data.columns:
            hum_col = 'Humidity-outside'
        elif location == 'living_room' and 'Humidity_DHT11' in data.columns:
            hum_col = 'Humidity_DHT11'
        elif location == 'desk' and 'humidity' in data.columns:
            hum_col = 'humidity'

        if hum_col:
            # Group by season and time of day
            season_tod_hum = data.groupby(['season', 'time_of_day'])[hum_col].mean().unstack()

            # Reorder seasons and time of day
            season_order = ['Winter', 'Spring', 'Summer', 'Fall']
            tod_order = ['Morning', 'Afternoon', 'Evening', 'Night']
            season_tod_hum = season_tod_hum.reindex(season_order)[tod_order]

            # Plot heatmap
            plt.imshow(season_tod_hum.values, cmap='viridis', aspect='auto')
            plt.colorbar(label='Humidity (%)')
            plt.xticks(range(len(tod_order)), tod_order)
            plt.yticks(range(len(season_order)), season_order)
            plt.title(f'{location.replace("_", " ").title()} Humidity by Season and Time of Day')
            break  # Only plot for the first location with humidity data

    # Plot 3: CO2 by season and time of day
    plt.subplot(3, 2, 3)
    for location, data in datasets.items():
        if not isinstance(data.index, pd.DatetimeIndex):
            data.index = pd.to_datetime(data.index)
            datasets[location] = data

    # Plot 4の修正部分：リサンプリング前にデータ型の確認と変換
    if ('bedroom' in datasets and 'Tempereture' in datasets['bedroom'].columns and
        'outside' in datasets and 'Temperature-outside' in datasets['outside'].columns):

        # データ型の変換を確実に行う
        bed_data = datasets['bedroom'].copy()
        bed_data['Tempereture'] = pd.to_numeric(bed_data['Tempereture'], errors='coerce')
        out_data = datasets['outside'].copy()
        out_data['Temperature-outside'] = pd.to_numeric(out_data['Temperature-outside'], errors='coerce')

        # リサンプリングを実行
        # Resample both datasets to hourly to align timestamps
        bed_hourly = bed_data.resample('h').mean(numeric_only=True)
        out_hourly = out_data.resample('h').mean(numeric_only=True)

        # Add season column back to bed_hourly by using the most common season for each hour
        # First create a series with the season for each hour
        bed_season_hourly = bed_data.resample('h')['season'].apply(lambda x: x.mode()[0] if not x.empty else None)

        # Add the season column to bed_hourly
        bed_hourly['season'] = bed_season_hourly

        # 以降のコードは同じ...

        # Merge datasets on timestamp
        merged = pd.merge(bed_hourly[['Tempereture', 'season']], 
                          out_hourly[['Temperature-outside']], 
                          left_index=True, right_index=True)

        # Calculate temperature difference
        merged['temp_diff'] = merged['Tempereture'] - merged['Temperature-outside']

        # Group by season
        seasonal_diff = merged.groupby('season')['temp_diff'].mean()

        # Reorder seasons
        season_order = ['Winter', 'Spring', 'Summer', 'Fall']
        seasonal_diff = seasonal_diff.reindex(season_order)

        # Plot bar chart
        plt.bar(range(len(seasonal_diff)), seasonal_diff.values)
        plt.title('Temperature Difference (Bedroom - Outside) by Season')
        plt.xlabel('Season')
        plt.ylabel('Temperature Difference (°C)')
        plt.xticks(range(len(seasonal_diff)), season_order)
        plt.grid(True)

    # Plot 5: CO2 levels by season and hour
    plt.subplot(3, 2, 5)
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            # Create pivot table
            pivot = data.pivot_table(
                values='CO2', 
                index='hour',
                columns='season',
                aggfunc='mean'
            )

            # Reorder seasons
            season_order = ['Winter', 'Spring', 'Summer', 'Fall']
            pivot = pivot[pivot.columns.intersection(season_order)]

            # Plot heatmap
            plt.imshow(pivot.values, cmap='viridis', aspect='auto')
            plt.colorbar(label='CO2 (ppm)')
            plt.title(f'{location.replace("_", " ").title()} CO2 by Season and Hour')
            plt.xlabel('Season')
            plt.ylabel('Hour of Day')
            plt.xticks(range(len(pivot.columns)), pivot.columns)
            plt.yticks(range(0, 24, 3), range(0, 24, 3))
            break  # Only plot for the first location with CO2 data

    # Plot 6: Absolute humidity by season and hour
    plt.subplot(3, 2, 6)
    for location, data in datasets.items():
        if 'Absolute_Humidity' in data.columns:
            # Create pivot table
            pivot = data.pivot_table(
                values='Absolute_Humidity', 
                index='hour',
                columns='season',
                aggfunc='mean'
            )

            # Reorder seasons
            season_order = ['Winter', 'Spring', 'Summer', 'Fall']
            pivot = pivot[pivot.columns.intersection(season_order)]

            # Plot heatmap
            plt.imshow(pivot.values, cmap='viridis', aspect='auto')
            plt.colorbar(label='Absolute Humidity (g/m³)')
            plt.title(f'{location.replace("_", " ").title()} Absolute Humidity by Season and Hour')
            plt.xlabel('Season')
            plt.ylabel('Hour of Day')
            plt.xticks(range(len(pivot.columns)), pivot.columns)
            plt.yticks(range(0, 24, 3), range(0, 24, 3))
            break  # Only plot for the first location with absolute humidity data

    plt.tight_layout()
    plt.savefig(f'{output_dir}/seasonal_trends.png', dpi=300)
    plt.close()

def analyze_pressure_trends(datasets, output_dir):
    """Analyze and visualize pressure trends by season and time of day."""
    plt.figure(figsize=(15, 10))

    # Plot 1: Pressure by hour of day
    plt.subplot(2, 2, 1)
    for location, data in datasets.items():
        press_col = None
        if location == 'bedroom' and 'Pressure' in data.columns:
            press_col = 'Pressure'
        elif location == 'outside' and 'Pressure-outside' in data.columns:
            press_col = 'Pressure-outside'
        elif location == 'desk' and 'pressure' in data.columns:
            press_col = 'pressure'

        if press_col:
            hourly_press = data.groupby('hour')[press_col].mean()
            plt.plot(hourly_press.index, hourly_press.values, label=location.replace('_', ' ').title())

    plt.title('Average Pressure by Hour of Day')
    plt.xlabel('Hour of Day')
    plt.ylabel('Pressure (hPa)')
    plt.xticks(range(0, 24, 2))
    plt.legend()
    plt.grid(True)

    # Plot 2: Pressure by season
    plt.subplot(2, 2, 2)
    for location, data in datasets.items():
        press_col = None
        if location == 'bedroom' and 'Pressure' in data.columns:
            press_col = 'Pressure'
        elif location == 'outside' and 'Pressure-outside' in data.columns:
            press_col = 'Pressure-outside'
        elif location == 'desk' and 'pressure' in data.columns:
            press_col = 'pressure'

        if press_col:
            seasonal_press = data.groupby('season')[press_col].mean()
            # Reorder seasons
            season_order = ['Winter', 'Spring', 'Summer', 'Fall']
            seasonal_press = seasonal_press.reindex(season_order)
            plt.bar(
                [i + 0.2 * list(datasets.keys()).index(location) for i in range(len(seasonal_press))], 
                seasonal_press.values, 
                width=0.2, 
                label=location.replace('_', ' ').title()
            )

    plt.title('Average Pressure by Season')
    plt.xlabel('Season')
    plt.ylabel('Pressure (hPa)')
    plt.xticks(range(4), season_order)
    plt.legend()
    plt.grid(True)

    # Plot 3: Pressure range by month
    plt.subplot(2, 2, 3)
    for location, data in datasets.items():
        press_col = None
        if location == 'bedroom' and 'Pressure' in data.columns:
            press_col = 'Pressure'
        elif location == 'outside' and 'Pressure-outside' in data.columns:
            press_col = 'Pressure-outside'
        elif location == 'desk' and 'pressure' in data.columns:
            press_col = 'pressure'

        if press_col:
            # Calculate monthly min, max, and mean
            monthly_min = data.groupby('month')[press_col].min()
            monthly_max = data.groupby('month')[press_col].max()
            monthly_mean = data.groupby('month')[press_col].mean()

            # Plot min, max, and mean
            # Use only the months for which we have data
            available_months = monthly_mean.index.values
            plt.plot(available_months, monthly_mean, 'o-', label=f"{location.replace('_', ' ').title()} - Mean")
            plt.fill_between(available_months, monthly_min, monthly_max, alpha=0.2)

    plt.title('Pressure Range by Month')
    plt.xlabel('Month')
    plt.ylabel('Pressure (hPa)')
    plt.xticks(range(1, 13))
    plt.legend()
    plt.grid(True)

    # Plot 4: Pressure by time of day and season
    plt.subplot(2, 2, 4)
    for location, data in datasets.items():
        press_col = None
        if location == 'bedroom' and 'Pressure' in data.columns:
            press_col = 'Pressure'
        elif location == 'outside' and 'Pressure-outside' in data.columns:
            press_col = 'Pressure-outside'
        elif location == 'desk' and 'pressure' in data.columns:
            press_col = 'pressure'

        if press_col:
            # Create pivot table
            pivot = data.pivot_table(
                values=press_col, 
                index='hour',
                columns='season',
                aggfunc='mean'
            )

            # Reorder seasons
            season_order = ['Winter', 'Spring', 'Summer', 'Fall']
            pivot = pivot[pivot.columns.intersection(season_order)]

            # Plot heatmap
            plt.imshow(pivot.values, cmap='viridis', aspect='auto')
            plt.colorbar(label='Pressure (hPa)')
            plt.title(f'{location.replace("_", " ").title()} Pressure by Season and Hour')
            plt.xlabel('Season')
            plt.ylabel('Hour of Day')
            plt.xticks(range(len(pivot.columns)), pivot.columns)
            plt.yticks(range(0, 24, 3), range(0, 24, 3))
            break  # Only plot for the first location with pressure data

    plt.tight_layout()
    plt.savefig(f'{output_dir}/pressure_trends.png', dpi=300)
    plt.close()

def analyze_monthly_trends(datasets, output_dir):
    """Analyze and visualize monthly trends for temperature, humidity, and CO2."""
    plt.figure(figsize=(15, 15))

    # Plot 1: Monthly temperature trends
    plt.subplot(3, 1, 1)
    for location, data in datasets.items():
        temp_col = None
        if location == 'bedroom' and 'Tempereture' in data.columns:
            temp_col = 'Tempereture'
        elif location == 'outside' and 'Temperature-outside' in data.columns:
            temp_col = 'Temperature-outside'
        elif location == 'living_room' and 'Temperature_DS18B20' in data.columns:
            temp_col = 'Temperature_DS18B20'
        elif location == 'desk' and 'temperature' in data.columns:
            temp_col = 'temperature'

        if temp_col:
            # Calculate monthly mean
            monthly_mean = data.groupby('month')[temp_col].mean()

            # Plot monthly mean
            # Use only the months for which we have data
            available_months = monthly_mean.index.values
            plt.plot(available_months, monthly_mean, 'o-', label=location.replace('_', ' ').title())

    plt.title('Monthly Temperature Trends')
    plt.xlabel('Month')
    plt.ylabel('Temperature (°C)')
    plt.xticks(range(1, 13))
    plt.legend()
    plt.grid(True)

    # Plot 2: Monthly humidity trends
    plt.subplot(3, 1, 2)
    for location, data in datasets.items():
        hum_col = None
        if location == 'bedroom' and 'Humidity' in data.columns:
            hum_col = 'Humidity'
        elif location == 'outside' and 'Humidity-outside' in data.columns:
            hum_col = 'Humidity-outside'
        elif location == 'living_room' and 'Humidity_DHT11' in data.columns:
            hum_col = 'Humidity_DHT11'
        elif location == 'desk' and 'humidity' in data.columns:
            hum_col = 'humidity'

        if hum_col:
            # Calculate monthly mean
            monthly_mean = data.groupby('month')[hum_col].mean()

            # Plot monthly mean
            # Use only the months for which we have data
            available_months = monthly_mean.index.values
            plt.plot(available_months, monthly_mean, 'o-', label=location.replace('_', ' ').title())

    plt.title('Monthly Humidity Trends')
    plt.xlabel('Month')
    plt.ylabel('Humidity (%)')
    plt.xticks(range(1, 13))
    plt.legend()
    plt.grid(True)

    # Plot 3: Monthly CO2 trends
    plt.subplot(3, 1, 3)
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            # Calculate monthly mean
            monthly_mean = data.groupby('month')['CO2'].mean()

            # Plot monthly mean
            # Use only the months for which we have data
            available_months = monthly_mean.index.values
            plt.plot(available_months, monthly_mean, 'o-', label=location.replace('_', ' ').title())

    plt.title('Monthly CO2 Trends')
    plt.xlabel('Month')
    plt.ylabel('CO2 (ppm)')
    plt.xticks(range(1, 13))
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/monthly_trends.png', dpi=300)
    plt.close()

    # Create monthly comparison table
    plt.figure(figsize=(15, 10))

    # Temperature comparison
    plt.subplot(2, 2, 1)
    temp_data = {}
    for location, data in datasets.items():
        temp_col = None
        if location == 'bedroom' and 'Tempereture' in data.columns:
            temp_col = 'Tempereture'
        elif location == 'outside' and 'Temperature-outside' in data.columns:
            temp_col = 'Temperature-outside'
        elif location == 'living_room' and 'Temperature_DS18B20' in data.columns:
            temp_col = 'Temperature_DS18B20'
        elif location == 'desk' and 'temperature' in data.columns:
            temp_col = 'temperature'

        if temp_col:
            temp_data[location] = data.groupby('month')[temp_col].mean()

    # Create DataFrame for comparison
    if temp_data:
        temp_df = pd.DataFrame(temp_data)

        # Plot heatmap
        sns.heatmap(temp_df, annot=True, fmt='.1f', cmap='viridis')
        plt.title('Monthly Temperature Comparison (°C)')
        plt.xlabel('Location')
        plt.ylabel('Month')

    # Humidity comparison
    plt.subplot(2, 2, 2)
    hum_data = {}
    for location, data in datasets.items():
        hum_col = None
        if location == 'bedroom' and 'Humidity' in data.columns:
            hum_col = 'Humidity'
        elif location == 'outside' and 'Humidity-outside' in data.columns:
            hum_col = 'Humidity-outside'
        elif location == 'living_room' and 'Humidity_DHT11' in data.columns:
            hum_col = 'Humidity_DHT11'
        elif location == 'desk' and 'humidity' in data.columns:
            hum_col = 'humidity'

        if hum_col:
            hum_data[location] = data.groupby('month')[hum_col].mean()

    # Create DataFrame for comparison
    if hum_data:
        hum_df = pd.DataFrame(hum_data)

        # Plot heatmap
        sns.heatmap(hum_df, annot=True, fmt='.1f', cmap='viridis')
        plt.title('Monthly Humidity Comparison (%)')
        plt.xlabel('Location')
        plt.ylabel('Month')

    # CO2 comparison
    plt.subplot(2, 2, 3)
    co2_data = {}
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            co2_data[location] = data.groupby('month')['CO2'].mean()

    # Create DataFrame for comparison
    if co2_data:
        co2_df = pd.DataFrame(co2_data)

        # Plot heatmap
        sns.heatmap(co2_df, annot=True, fmt='.0f', cmap='viridis')
        plt.title('Monthly CO2 Comparison (ppm)')
        plt.xlabel('Location')
        plt.ylabel('Month')

    # Pressure comparison
    plt.subplot(2, 2, 4)
    press_data = {}
    for location, data in datasets.items():
        press_col = None
        if location == 'bedroom' and 'Pressure' in data.columns:
            press_col = 'Pressure'
        elif location == 'outside' and 'Pressure-outside' in data.columns:
            press_col = 'Pressure-outside'
        elif location == 'desk' and 'pressure' in data.columns:
            press_col = 'pressure'

        if press_col:
            press_data[location] = data.groupby('month')[press_col].mean()

    # Create DataFrame for comparison
    if press_data:
        press_df = pd.DataFrame(press_data)

        # Plot heatmap
        sns.heatmap(press_df, annot=True, fmt='.1f', cmap='viridis')
        plt.title('Monthly Pressure Comparison (hPa)')
        plt.xlabel('Location')
        plt.ylabel('Month')

    plt.tight_layout()
    plt.savefig(f'{output_dir}/monthly_comparison.png', dpi=300)
    plt.close()

def ml_analysis_temperature(datasets, output_dir):
    """Use machine learning to analyze temperature patterns."""
    plt.figure(figsize=(15, 10))

    # Prepare data for clustering
    for location, data in datasets.items():
        temp_col = None
        if location == 'bedroom' and 'Tempereture' in data.columns:
            temp_col = 'Tempereture'
        elif location == 'outside' and 'Temperature-outside' in data.columns:
            temp_col = 'Temperature-outside'
        elif location == 'living_room' and 'Temperature_DS18B20' in data.columns:
            temp_col = 'Temperature_DS18B20'
        elif location == 'desk' and 'temperature' in data.columns:
            temp_col = 'temperature'

        if temp_col:
            # Create features for clustering
            features = data[[temp_col, 'hour', 'month']].copy()
            features = features.dropna()

            if len(features) > 0:
                # Standardize features
                scaler = StandardScaler()
                features_scaled = scaler.fit_transform(features)

                # Apply K-means clustering
                kmeans = KMeans(n_clusters=4, random_state=42)
                clusters = kmeans.fit_predict(features_scaled)

                # Add cluster labels to original data
                features['cluster'] = clusters

                # Plot 1: Temperature by hour, colored by cluster
                plt.subplot(2, 2, 1)
                for cluster in range(4):
                    cluster_data = features[features['cluster'] == cluster]
                    plt.scatter(cluster_data['hour'], cluster_data[temp_col], 
                                label=f'Cluster {cluster}', alpha=0.5)

                plt.title(f'{location.replace("_", " ").title()} Temperature Clusters by Hour')
                plt.xlabel('Hour of Day')
                plt.ylabel('Temperature (°C)')
                plt.legend()
                plt.grid(True)

                # Plot 2: Temperature by month, colored by cluster
                plt.subplot(2, 2, 2)
                for cluster in range(4):
                    cluster_data = features[features['cluster'] == cluster]
                    plt.scatter(cluster_data['month'], cluster_data[temp_col], 
                                label=f'Cluster {cluster}', alpha=0.5)

                plt.title(f'{location.replace("_", " ").title()} Temperature Clusters by Month')
                plt.xlabel('Month')
                plt.ylabel('Temperature (°C)')
                plt.legend()
                plt.grid(True)

                # Plot 3: Cluster centers
                plt.subplot(2, 2, 3)
                centers = scaler.inverse_transform(kmeans.cluster_centers_)

                # Create bar chart of cluster centers
                x = np.arange(3)
                width = 0.2

                for i in range(4):
                    plt.bar(x + i*width, centers[i], width, label=f'Cluster {i}')

                plt.title(f'{location.replace("_", " ").title()} Cluster Centers')
                plt.xlabel('Feature')
                plt.ylabel('Value')
                plt.xticks(x + width*1.5, [temp_col, 'Hour', 'Month'])
                plt.legend()
                plt.grid(True)

                # Plot 4: PCA visualization of clusters
                plt.subplot(2, 2, 4)
                pca = PCA(n_components=2)
                features_pca = pca.fit_transform(features_scaled)

                for cluster in range(4):
                    cluster_data = features_pca[clusters == cluster]
                    plt.scatter(cluster_data[:, 0], cluster_data[:, 1], 
                                label=f'Cluster {cluster}', alpha=0.5)

                plt.title(f'{location.replace("_", " ").title()} PCA Visualization of Temperature Clusters')
                plt.xlabel('Principal Component 1')
                plt.ylabel('Principal Component 2')
                plt.legend()
                plt.grid(True)

                # Save plot
                plt.tight_layout()
                plt.savefig(f'{output_dir}/temperature_ml_{location}.png', dpi=300)
                plt.close()

                # Create a new figure for the next location
                if location != list(datasets.keys())[-1]:
                    plt.figure(figsize=(15, 10))

def ml_analysis_co2(datasets, output_dir):
    """Use machine learning to analyze CO2 patterns."""
    plt.figure(figsize=(15, 10))

    # Prepare data for clustering
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            # Create features for clustering
            features = data[['CO2', 'hour', 'month']].copy()
            features = features.dropna()

            if len(features) > 0:
                # Standardize features
                scaler = StandardScaler()
                features_scaled = scaler.fit_transform(features)

                # Apply K-means clustering
                kmeans = KMeans(n_clusters=4, random_state=42)
                clusters = kmeans.fit_predict(features_scaled)

                # Add cluster labels to original data
                features['cluster'] = clusters

                # Plot 1: CO2 by hour, colored by cluster
                plt.subplot(2, 2, 1)
                for cluster in range(4):
                    cluster_data = features[features['cluster'] == cluster]
                    plt.scatter(cluster_data['hour'], cluster_data['CO2'], 
                                label=f'Cluster {cluster}', alpha=0.5)

                plt.title(f'{location.replace("_", " ").title()} CO2 Clusters by Hour')
                plt.xlabel('Hour of Day')
                plt.ylabel('CO2 (ppm)')
                plt.legend()
                plt.grid(True)

                # Plot 2: CO2 by month, colored by cluster
                plt.subplot(2, 2, 2)
                for cluster in range(4):
                    cluster_data = features[features['cluster'] == cluster]
                    plt.scatter(cluster_data['month'], cluster_data['CO2'], 
                                label=f'Cluster {cluster}', alpha=0.5)

                plt.title(f'{location.replace("_", " ").title()} CO2 Clusters by Month')
                plt.xlabel('Month')
                plt.ylabel('CO2 (ppm)')
                plt.legend()
                plt.grid(True)

                # Plot 3: Cluster centers
                plt.subplot(2, 2, 3)
                centers = scaler.inverse_transform(kmeans.cluster_centers_)

                # Create bar chart of cluster centers
                x = np.arange(3)
                width = 0.2

                for i in range(4):
                    plt.bar(x + i*width, centers[i], width, label=f'Cluster {i}')

                plt.title(f'{location.replace("_", " ").title()} Cluster Centers')
                plt.xlabel('Feature')
                plt.ylabel('Value')
                plt.xticks(x + width*1.5, ['CO2', 'Hour', 'Month'])
                plt.legend()
                plt.grid(True)

                # Plot 4: PCA visualization of clusters
                plt.subplot(2, 2, 4)
                pca = PCA(n_components=2)
                features_pca = pca.fit_transform(features_scaled)

                for cluster in range(4):
                    cluster_data = features_pca[clusters == cluster]
                    plt.scatter(cluster_data[:, 0], cluster_data[:, 1], 
                                label=f'Cluster {cluster}', alpha=0.5)

                plt.title(f'{location.replace("_", " ").title()} PCA Visualization of CO2 Clusters')
                plt.xlabel('Principal Component 1')
                plt.ylabel('Principal Component 2')
                plt.legend()
                plt.grid(True)

                # Save plot
                plt.tight_layout()
                plt.savefig(f'{output_dir}/co2_ml_{location}.png', dpi=300)
                plt.close()

                # Create a new figure for the next location
                if location != list(datasets.keys())[-1]:
                    plt.figure(figsize=(15, 10))

# Run the analysis
if __name__ == "__main__":
    analyze_environmental_data()
