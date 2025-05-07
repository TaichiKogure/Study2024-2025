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
from statsmodels.tsa.seasonal import seasonal_decompose
from scipy import stats
import matplotlib.ticker as ticker
from sklearn.ensemble import IsolationForest

# Constants for time-based analysis
class TimeConstants:
    SEASONS = {
        'Spring': range(3, 6),
        'Summer': range(6, 9),
        'Fall': range(9, 12),
        'Winter': list(range(12, 13)) + list(range(1, 3))
    }

    TIME_OF_DAY = {
        'Morning': range(5, 12),
        'Afternoon': range(12, 17),
        'Evening': range(17, 22),
        'Night': list(range(22, 24)) + list(range(0, 5))
    }

class CO2DataAnalyzer:
    def __init__(self, data_files: Dict[str, str], output_dir: str):
        self.data_files = data_files
        self.output_dir = output_dir
        self.datasets = {}

        # Set style for plots
        plt.style.use('seaborn-v0_8-whitegrid')
        sns.set_palette("viridis")

        # Create output directory
        os.makedirs(output_dir, exist_ok=True)

    # Define seasons
    def get_season(self, month: int) -> str:
        try:
            return next(season for season, months in TimeConstants.SEASONS.items()
                       if month in months)
        except StopIteration:
            # Default to Winter if month doesn't match any defined season
            return 'Winter'

    # Define time of day
    def get_time_of_day(self, hour: int) -> str:
        try:
            return next(period for period, hours in TimeConstants.TIME_OF_DAY.items()
                       if hour in hours)
        except StopIteration:
            # Default to Night if hour doesn't match any defined time period
            return 'Night'

    # Load and preprocess data
    def load_data(self, file_path: str, date_column: str = 'current_time') -> pd.DataFrame:
        print(f"Loading data from {file_path}...")
        data = pd.read_csv(file_path)

        # Convert date column to datetime
        data[date_column] = pd.to_datetime(data[date_column])

        # Set date column as index
        data.set_index(date_column, inplace=True)

        # Add time-related features
        data['month'] = data.index.month
        data['day'] = data.index.day
        data['hour'] = data.index.hour
        data['weekday'] = data.index.weekday
        data['is_weekend'] = data['weekday'].apply(lambda x: 1 if x >= 5 else 0)
        data['date'] = data.index.date
        data['week'] = data.index.isocalendar().week

        # Add season and time of day
        data['season'] = data['month'].apply(self.get_season)
        data['time_of_day'] = data['hour'].apply(self.get_time_of_day)

        # Handle missing values and special values
        # Replace 0 and 500 in CO2 column with NaN
        if 'CO2' in data.columns:
            data['CO2'] = data['CO2'].replace([0, 500], np.nan)
            # Convert 'None' strings to NaN
            data['CO2'] = pd.to_numeric(data['CO2'], errors='coerce')

        print(f"Data loaded. Shape: {data.shape}")
        return data

    def load_all_datasets(self):
        for location, file_path in self.data_files.items():
            try:
                data = self.load_data(file_path)
                self.datasets[location] = data
            except Exception as e:
                print(f"Error loading {file_path}: {str(e)}")
                import traceback
                traceback.print_exc()

    def analyze(self):
        """Run all analysis functions"""
        self.load_all_datasets()

        # Create analysis functions dictionary
        analysis_functions = {
            "daily_trends": analyze_daily_trends,
            "monthly_trends": analyze_monthly_trends,
            "hourly_patterns": analyze_hourly_patterns,
            "weekday_weekend_patterns": analyze_weekday_weekend_patterns,
            "seasonal_trends": analyze_seasonal_trends,
            "basic_statistics": analyze_basic_statistics,
            "correlation_analysis": analyze_correlation_analysis,
            "outlier_detection": analyze_outlier_detection,
            "frequency_distribution": analyze_frequency_distribution,
            "time_series_decomposition": analyze_time_series_decomposition,
            "moving_averages": analyze_moving_averages,
            "room_comparison": analyze_room_comparison,
            "peak_detection": analyze_peak_detection,
            "co2_concentration_heatmap": analyze_co2_concentration_heatmap,
            "forecasting_patterns": analyze_forecasting_patterns
        }

        # Run each analysis function
        for name, func in analysis_functions.items():
            try:
                print(f"Running {name}...")
                func(self.datasets, self.output_dir)
                print(f"Completed {name}")
            except Exception as e:
                print(f"Error in {name}: {e}")

def analyze_daily_trends(datasets, output_dir):
    """Analyze and visualize CO2 trends by day of week and hour of day."""
    plt.figure(figsize=(15, 10))

    # Plot 1: Average CO2 by day of week
    plt.subplot(2, 1, 1)
    days = ['Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday', 'Sunday']

    has_data_plot1 = False
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            daily_co2 = data.groupby('weekday')['CO2'].mean()
            if not daily_co2.empty:
                plt.plot(daily_co2.index, daily_co2.values, marker='o', label=location.replace('_', ' ').title())
                has_data_plot1 = True

    plt.title('Average CO2 by Day of Week')
    plt.xlabel('Day of Week')
    plt.ylabel('CO2 (ppm)')
    plt.xticks(range(7), days)
    if has_data_plot1:
        plt.legend()
    plt.grid(True)

    # Plot 2: Average CO2 by hour of day
    plt.subplot(2, 1, 2)
    has_data_plot2 = False
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            hourly_co2 = data.groupby('hour')['CO2'].mean()
            if not hourly_co2.empty:
                plt.plot(hourly_co2.index, hourly_co2.values, marker='o', label=location.replace('_', ' ').title())
                has_data_plot2 = True

    plt.title('Average CO2 by Hour of Day')
    plt.xlabel('Hour of Day')
    plt.ylabel('CO2 (ppm)')
    plt.xticks(range(0, 24, 2))
    if has_data_plot2:
        plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/daily_trends.png', dpi=300)
    plt.close()

def analyze_monthly_trends(datasets, output_dir):
    """Analyze and visualize CO2 trends by month."""
    plt.figure(figsize=(15, 10))

    # Plot 1: Average CO2 by month
    plt.subplot(2, 1, 1)
    months = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']

    has_data_plot1 = False
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            monthly_co2 = data.groupby('month')['CO2'].mean()
            if not monthly_co2.empty:
                plt.plot(monthly_co2.index, monthly_co2.values, marker='o', label=location.replace('_', ' ').title())
                has_data_plot1 = True

    plt.title('Average CO2 by Month')
    plt.xlabel('Month')
    plt.ylabel('CO2 (ppm)')
    plt.xticks(range(1, 13), months)
    if has_data_plot1:
        plt.legend()
    plt.grid(True)

    # Plot 2: CO2 trend over time (monthly average)
    plt.subplot(2, 1, 2)
    has_data_plot2 = False
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            # Resample to monthly data
            monthly_data = data.resample('ME').mean(numeric_only=True)
            if 'CO2' in monthly_data.columns and not monthly_data['CO2'].empty:
                plt.plot(monthly_data.index, monthly_data['CO2'], marker='o', label=location.replace('_', ' ').title())
                has_data_plot2 = True

    plt.title('Monthly Average CO2 Over Time')
    plt.xlabel('Date')
    plt.ylabel('CO2 (ppm)')
    if has_data_plot2:
        plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/monthly_trends.png', dpi=300)
    plt.close()

def analyze_hourly_patterns(datasets, output_dir):
    """Analyze and visualize hourly CO2 patterns by different days."""
    plt.figure(figsize=(15, 15))

    # Plot 1: Hourly patterns for weekdays
    plt.subplot(3, 1, 1)
    has_data_plot1 = False
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            weekday_data = data[data['is_weekend'] == 0]
            hourly_co2 = weekday_data.groupby('hour')['CO2'].mean()
            if not hourly_co2.empty:
                plt.plot(hourly_co2.index, hourly_co2.values, marker='o', label=location.replace('_', ' ').title())
                has_data_plot1 = True

    plt.title('Hourly CO2 Patterns on Weekdays')
    plt.xlabel('Hour of Day')
    plt.ylabel('CO2 (ppm)')
    plt.xticks(range(0, 24, 2))
    if has_data_plot1:
        plt.legend()
    plt.grid(True)

    # Plot 2: Hourly patterns for weekends
    plt.subplot(3, 1, 2)
    has_data_plot2 = False
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            weekend_data = data[data['is_weekend'] == 1]
            hourly_co2 = weekend_data.groupby('hour')['CO2'].mean()
            if not hourly_co2.empty:
                plt.plot(hourly_co2.index, hourly_co2.values, marker='o', label=location.replace('_', ' ').title())
                has_data_plot2 = True

    plt.title('Hourly CO2 Patterns on Weekends')
    plt.xlabel('Hour of Day')
    plt.ylabel('CO2 (ppm)')
    plt.xticks(range(0, 24, 2))
    if has_data_plot2:
        plt.legend()
    plt.grid(True)

    # Plot 3: Difference between weekday and weekend
    plt.subplot(3, 1, 3)
    has_data_plot3 = False
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            weekday_data = data[data['is_weekend'] == 0]
            weekend_data = data[data['is_weekend'] == 1]

            weekday_hourly = weekday_data.groupby('hour')['CO2'].mean()
            weekend_hourly = weekend_data.groupby('hour')['CO2'].mean()

            if not weekday_hourly.empty and not weekend_hourly.empty:
                # Calculate difference (weekend - weekday)
                diff = weekend_hourly - weekday_hourly
                plt.plot(diff.index, diff.values, marker='o', label=location.replace('_', ' ').title())
                has_data_plot3 = True

    plt.title('Difference in CO2 Levels (Weekend - Weekday)')
    plt.xlabel('Hour of Day')
    plt.ylabel('CO2 Difference (ppm)')
    plt.xticks(range(0, 24, 2))
    plt.axhline(y=0, color='r', linestyle='-', alpha=0.3)
    if has_data_plot3:
        plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/hourly_patterns.png', dpi=300)
    plt.close()

def analyze_weekday_weekend_patterns(datasets, output_dir):
    """Analyze and visualize CO2 patterns between weekdays and weekends."""
    plt.figure(figsize=(15, 15))

    # Plot 1: Average CO2 by weekday/weekend
    plt.subplot(3, 1, 1)
    has_data_plot1 = False
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            weekend_co2 = data.groupby('is_weekend')['CO2'].mean()
            if not weekend_co2.empty:
                plt.bar(
                    [i + 0.2 * list(datasets.keys()).index(location) for i in range(len(weekend_co2))], 
                    weekend_co2.values, 
                    width=0.2, 
                    label=location.replace('_', ' ').title()
                )
                has_data_plot1 = True

    plt.title('Average CO2 by Weekday/Weekend')
    plt.xlabel('Day Type')
    plt.ylabel('CO2 (ppm)')
    plt.xticks([0, 1], ['Weekday', 'Weekend'])
    if has_data_plot1:
        plt.legend()
    plt.grid(True)

    # Plot 2: Daily pattern for each day of the week
    plt.subplot(3, 1, 2)
    days = ['Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday', 'Sunday']
    has_data_plot2 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns:
            for day in range(7):
                day_data = data[data['weekday'] == day]
                if not day_data.empty:
                    hourly_co2 = day_data.groupby('hour')['CO2'].mean()
                    if not hourly_co2.empty:
                        plt.plot(hourly_co2.index, hourly_co2.values, label=f"{days[day]}" if location == list(datasets.keys())[0] else "", alpha=0.7)
                        if location == list(datasets.keys())[0]:  # Only count labels from the first location
                            has_data_plot2 = True

    plt.title('Hourly CO2 Patterns by Day of Week')
    plt.xlabel('Hour of Day')
    plt.ylabel('CO2 (ppm)')
    plt.xticks(range(0, 24, 2))
    if has_data_plot2:
        plt.legend()
    plt.grid(True)

    # Plot 3: Box plot of CO2 by day of week
    plt.subplot(3, 1, 3)

    day_data = []
    day_labels = []

    for location, data in datasets.items():
        if 'CO2' in data.columns:
            for day in range(7):
                co2_data = data[data['weekday'] == day]['CO2'].dropna()
                if not co2_data.empty:
                    day_data.append(co2_data)
                    day_labels.append(f"{days[day]} ({location})")

    if day_data:  # Only create boxplot if there's data
        plt.boxplot(day_data, tick_labels=day_labels, vert=True)
        plt.title('CO2 Distribution by Day of Week')
        plt.xlabel('Day of Week')
        plt.ylabel('CO2 (ppm)')
        plt.xticks(rotation=45)
        plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/weekday_weekend_patterns.png', dpi=300)
    plt.close()

def analyze_seasonal_trends(datasets, output_dir):
    """Analyze and visualize seasonal CO2 trends."""
    plt.figure(figsize=(15, 15))

    # Plot 1: Average CO2 by season
    plt.subplot(3, 1, 1)
    season_order = ['Winter', 'Spring', 'Summer', 'Fall']
    has_data_plot1 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns:
            seasonal_co2 = data.groupby('season')['CO2'].mean()
            # Reorder seasons
            seasonal_co2 = seasonal_co2.reindex(season_order)
            if not seasonal_co2.empty and not seasonal_co2.isna().all():
                plt.bar(
                    [i + 0.2 * list(datasets.keys()).index(location) for i in range(len(seasonal_co2))], 
                    seasonal_co2.values, 
                    width=0.2, 
                    label=location.replace('_', ' ').title()
                )
                has_data_plot1 = True

    plt.title('Average CO2 by Season')
    plt.xlabel('Season')
    plt.ylabel('CO2 (ppm)')
    plt.xticks(range(4), season_order)
    if has_data_plot1:
        plt.legend()
    plt.grid(True)

    # Plot 2: Hourly patterns by season
    plt.subplot(3, 1, 2)
    has_data_plot2 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns:
            for season in season_order:
                season_data = data[data['season'] == season]
                if not season_data.empty:
                    hourly_co2 = season_data.groupby('hour')['CO2'].mean()
                    if not hourly_co2.empty and not hourly_co2.isna().all():
                        plt.plot(hourly_co2.index, hourly_co2.values, label=f"{season}" if location == list(datasets.keys())[0] else "", alpha=0.7)
                        if location == list(datasets.keys())[0] and f"{season}":  # Only count labels from the first location
                            has_data_plot2 = True

    plt.title('Hourly CO2 Patterns by Season')
    plt.xlabel('Hour of Day')
    plt.ylabel('CO2 (ppm)')
    plt.xticks(range(0, 24, 2))
    if has_data_plot2:
        plt.legend()
    plt.grid(True)

    # Plot 3: Box plot of CO2 by season
    plt.subplot(3, 1, 3)

    season_data = []
    season_labels = []

    for location, data in datasets.items():
        if 'CO2' in data.columns:
            for season in season_order:
                co2_data = data[data['season'] == season]['CO2'].dropna()
                if not co2_data.empty:
                    season_data.append(co2_data)
                    season_labels.append(f"{season} ({location})")

    if season_data:  # Only create boxplot if there's data
        plt.boxplot(season_data, tick_labels=season_labels, vert=True)
        plt.title('CO2 Distribution by Season')
        plt.xlabel('Season')
        plt.ylabel('CO2 (ppm)')
        plt.xticks(rotation=45)
        plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/seasonal_trends.png', dpi=300)
    plt.close()

def analyze_basic_statistics(datasets, output_dir):
    """Analyze and visualize basic statistics of CO2 data."""
    plt.figure(figsize=(15, 15))

    # Plot 1: Histogram of CO2 values
    plt.subplot(3, 1, 1)
    has_data_plot1 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns:
            co2_data = data['CO2'].dropna()
            if not co2_data.empty:
                plt.hist(co2_data, bins=50, alpha=0.5, label=location.replace('_', ' ').title())
                has_data_plot1 = True

    plt.title('Distribution of CO2 Values')
    plt.xlabel('CO2 (ppm)')
    plt.ylabel('Frequency')
    if has_data_plot1:
        plt.legend()
    plt.grid(True)

    # Plot 2: Box plot of CO2 values
    plt.subplot(3, 1, 2)

    co2_data = []
    labels = []

    for location, data in datasets.items():
        if 'CO2' in data.columns:
            data_to_plot = data['CO2'].dropna()
            if not data_to_plot.empty:
                co2_data.append(data_to_plot)
                labels.append(location.replace('_', ' ').title())

    if co2_data:  # Only create boxplot if there's data
        plt.boxplot(co2_data, tick_labels=labels, vert=True)
        plt.title('CO2 Distribution by Location')
        plt.xlabel('Location')
        plt.ylabel('CO2 (ppm)')
        plt.grid(True)

    # Plot 3: Table of statistics
    plt.subplot(3, 1, 3)
    plt.axis('off')

    stats_data = []
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            co2_stats = data['CO2'].describe()
            stats_data.append([
                location.replace('_', ' ').title(),
                f"{co2_stats['count']:.0f}",
                f"{co2_stats['mean']:.2f}",
                f"{co2_stats['std']:.2f}",
                f"{co2_stats['min']:.2f}",
                f"{co2_stats['25%']:.2f}",
                f"{co2_stats['50%']:.2f}",
                f"{co2_stats['75%']:.2f}",
                f"{co2_stats['max']:.2f}"
            ])

    table = plt.table(
        cellText=stats_data,
        colLabels=['Location', 'Count', 'Mean', 'Std', 'Min', '25%', '50%', '75%', 'Max'],
        loc='center',
        cellLoc='center'
    )
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 1.5)
    plt.title('CO2 Statistics by Location')

    plt.tight_layout()
    plt.savefig(f'{output_dir}/basic_statistics.png', dpi=300)
    plt.close()

def analyze_correlation_analysis(datasets, output_dir):
    """Analyze and visualize correlations between CO2 and other variables."""
    plt.figure(figsize=(15, 15))

    # Plot 1: Correlation between CO2 and time-related variables
    plt.subplot(3, 1, 1)

    # Define variables outside the loop to ensure they're always available
    variables = ['Hour', 'Weekday', 'Month', 'Is Weekend']

    for location, data in datasets.items():
        if 'CO2' in data.columns:
            # Calculate correlations
            corr_hour = data['CO2'].corr(data['hour'])
            corr_weekday = data['CO2'].corr(data['weekday'])
            corr_month = data['CO2'].corr(data['month'])
            corr_weekend = data['CO2'].corr(data['is_weekend'])

            # Plot correlations
            correlations = [corr_hour, corr_weekday, corr_month, corr_weekend]

            x = range(len(variables))
            plt.bar([i + 0.2 * list(datasets.keys()).index(location) for i in x], 
                   correlations, width=0.2, label=location.replace('_', ' ').title())

    plt.title('Correlation between CO2 and Time Variables')
    plt.xlabel('Variable')
    plt.ylabel('Correlation Coefficient')
    plt.xticks(range(len(variables)), variables)
    plt.axhline(y=0, color='r', linestyle='-', alpha=0.3)
    plt.legend()
    plt.grid(True)

    # Plot 2: Correlation between CO2 and other environmental variables
    plt.subplot(3, 1, 2)

    for location, data in datasets.items():
        if 'CO2' in data.columns:
            # Get numerical columns excluding time-related ones
            num_cols = data.select_dtypes(include=[np.number]).columns
            exclude_cols = ['CO2', 'hour', 'weekday', 'month', 'day', 'is_weekend', 'week']
            env_cols = [col for col in num_cols if col not in exclude_cols]

            if env_cols:
                correlations = [data['CO2'].corr(data[col]) for col in env_cols]

                x = range(len(env_cols))
                plt.bar([i + 0.2 * list(datasets.keys()).index(location) for i in x], 
                       correlations, width=0.2, label=location.replace('_', ' ').title())

                plt.title('Correlation between CO2 and Environmental Variables')
                plt.xlabel('Variable')
                plt.ylabel('Correlation Coefficient')
                plt.xticks(range(len(env_cols)), [col.replace('_', ' ').title() for col in env_cols], rotation=45)
                plt.axhline(y=0, color='r', linestyle='-', alpha=0.3)
                plt.legend()
                plt.grid(True)

    # Plot 3: Scatter plot of CO2 vs most correlated variable
    plt.subplot(3, 1, 3)
    has_data_plot3 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns and not data['CO2'].empty:
            # Get numerical columns excluding time-related ones
            num_cols = data.select_dtypes(include=[np.number]).columns
            exclude_cols = ['CO2', 'hour', 'weekday', 'month', 'day', 'is_weekend', 'week']
            env_cols = [col for col in num_cols if col not in exclude_cols]

            if env_cols:
                # Find most correlated variable
                valid_cols = []
                valid_correlations = []
                for col in env_cols:
                    if not data[col].empty and not data[col].isna().all():
                        try:
                            corr = abs(data['CO2'].corr(data[col]))
                            if not np.isnan(corr):
                                valid_cols.append(col)
                                valid_correlations.append(corr)
                        except:
                            pass

                if valid_cols:
                    most_corr_idx = np.argmax(valid_correlations)
                    most_corr_var = valid_cols[most_corr_idx]

                    # Plot scatter
                    plt.scatter(data[most_corr_var], data['CO2'], alpha=0.5, label=location.replace('_', ' ').title())
                    has_data_plot3 = True

                    plt.title(f'CO2 vs {most_corr_var.replace("_", " ").title()}')
                    plt.xlabel(most_corr_var.replace('_', ' ').title())
                    plt.ylabel('CO2 (ppm)')

    if has_data_plot3:
        plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/correlation_analysis.png', dpi=300)
    plt.close()

def analyze_outlier_detection(datasets, output_dir):
    """Detect and visualize outliers in CO2 data."""
    plt.figure(figsize=(15, 15))

    # Plot 1: Box plot with outliers highlighted
    plt.subplot(3, 1, 1)
    has_data_plot1 = False

    for i, (location, data) in enumerate(datasets.items()):
        if 'CO2' in data.columns:
            co2_data = data['CO2'].dropna()
            if not co2_data.empty:
                has_data_plot1 = True

                # Calculate outlier boundaries
                q1 = co2_data.quantile(0.25)
                q3 = co2_data.quantile(0.75)
                iqr = q3 - q1
                lower_bound = q1 - 1.5 * iqr
                upper_bound = q3 + 1.5 * iqr

                # Identify outliers
                outliers = co2_data[(co2_data < lower_bound) | (co2_data > upper_bound)]
                non_outliers = co2_data[(co2_data >= lower_bound) & (co2_data <= upper_bound)]

                # Plot box plot
                plt.boxplot([co2_data], positions=[i], widths=0.6)

                # Plot outliers
                if not outliers.empty:
                    plt.scatter([i] * len(outliers), outliers, color='red', marker='o', alpha=0.5)

    if has_data_plot1:
        plt.title('CO2 Outliers by Location')
        plt.xlabel('Location')
        plt.ylabel('CO2 (ppm)')
        plt.xticks(range(len(datasets)), [loc.replace('_', ' ').title() for loc in datasets.keys()])
        plt.grid(True)

    # Plot 2: Time series with outliers highlighted
    plt.subplot(3, 1, 2)
    has_data_plot2 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns and not data['CO2'].empty:
            co2_data = data['CO2']
            has_data_plot2 = True

            # Calculate outlier boundaries
            q1 = co2_data.quantile(0.25)
            q3 = co2_data.quantile(0.75)
            iqr = q3 - q1
            lower_bound = q1 - 1.5 * iqr
            upper_bound = q3 + 1.5 * iqr

            # Identify outliers
            outliers = data[((co2_data < lower_bound) | (co2_data > upper_bound)) & (~co2_data.isna())]
            non_outliers = data[((co2_data >= lower_bound) & (co2_data <= upper_bound)) & (~co2_data.isna())]

            # Plot time series
            plt.plot(non_outliers.index, non_outliers['CO2'], label=location.replace('_', ' ').title(), alpha=0.7)

            # Plot outliers
            if not outliers.empty:
                plt.scatter(outliers.index, outliers['CO2'], color='red', marker='o', alpha=0.5)

    plt.title('CO2 Time Series with Outliers Highlighted')
    plt.xlabel('Date')
    plt.ylabel('CO2 (ppm)')
    if has_data_plot2:
        plt.legend()
    plt.grid(True)

    # Plot 3: Isolation Forest outlier detection
    plt.subplot(3, 1, 3)
    has_data_plot3 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns:
            co2_data = data['CO2'].dropna()
            if len(co2_data) > 10:  # Need enough data for Isolation Forest
                has_data_plot3 = True
                co2_array = co2_data.values.reshape(-1, 1)

                # Apply Isolation Forest
                iso_forest = IsolationForest(contamination=0.05, random_state=42)
                outlier_labels = iso_forest.fit_predict(co2_array)

                # Get indices of outliers
                outlier_indices = np.where(outlier_labels == -1)[0]

                # Plot non-outliers
                plt.plot(range(len(co2_data)), co2_data.values, label=location.replace('_', ' ').title(), alpha=0.7)

                # Plot outliers
                if len(outlier_indices) > 0:
                    plt.scatter(outlier_indices, co2_data.iloc[outlier_indices], color='red', marker='o', alpha=0.5)

    plt.title('CO2 Outliers Detected by Isolation Forest')
    plt.xlabel('Data Point Index')
    plt.ylabel('CO2 (ppm)')
    if has_data_plot3:
        plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/outlier_detection.png', dpi=300)
    plt.close()

def analyze_frequency_distribution(datasets, output_dir):
    """Analyze and visualize the frequency distribution of CO2 values."""
    plt.figure(figsize=(15, 15))

    # Plot 1: Histogram with KDE
    plt.subplot(3, 1, 1)
    has_data_plot1 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns:
            co2_data = data['CO2'].dropna()
            if not co2_data.empty and len(co2_data) > 1:
                sns.histplot(co2_data, kde=True, label=location.replace('_', ' ').title(), alpha=0.5)
                has_data_plot1 = True

    plt.title('Frequency Distribution of CO2 Values')
    plt.xlabel('CO2 (ppm)')
    plt.ylabel('Frequency')
    if has_data_plot1:
        plt.legend()
    plt.grid(True)

    # Plot 2: Cumulative distribution
    plt.subplot(3, 1, 2)
    has_data_plot2 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns:
            co2_data = data['CO2'].dropna()
            if not co2_data.empty and len(co2_data) > 1:
                co2_sorted = np.sort(co2_data.values)
                p = 1. * np.arange(len(co2_sorted)) / (len(co2_sorted) - 1)
                plt.plot(co2_sorted, p, label=location.replace('_', ' ').title())
                has_data_plot2 = True

    plt.title('Cumulative Distribution of CO2 Values')
    plt.xlabel('CO2 (ppm)')
    plt.ylabel('Cumulative Probability')
    if has_data_plot2:
        plt.legend()
    plt.grid(True)

    # Plot 3: Probability density function
    plt.subplot(3, 1, 3)
    has_data_plot3 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns:
            co2_data = data['CO2'].dropna()
            if not co2_data.empty and len(co2_data) > 10:  # Need enough data for KDE
                try:
                    kde = stats.gaussian_kde(co2_data)
                    x = np.linspace(co2_data.min(), co2_data.max(), 1000)
                    plt.plot(x, kde(x), label=location.replace('_', ' ').title())
                    has_data_plot3 = True
                except:
                    pass  # Skip if KDE fails

    plt.title('Probability Density Function of CO2 Values')
    plt.xlabel('CO2 (ppm)')
    plt.ylabel('Density')
    if has_data_plot3:
        plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/frequency_distribution.png', dpi=300)
    plt.close()

def analyze_time_series_decomposition(datasets, output_dir):
    """Decompose CO2 time series into trend, seasonal, and residual components."""
    plt.figure(figsize=(15, 20))

    plot_idx = 1

    for location, data in datasets.items():
        if 'CO2' in data.columns:
            # Resample to daily data to reduce noise
            daily_data = data.resample('D').mean(numeric_only=True)

            # Check if we have enough data for decomposition
            if len(daily_data) >= 14:  # Need at least 2 weeks of data
                try:
                    # Handle missing values by interpolation
                    co2_series = daily_data['CO2'].interpolate(method='linear').dropna()

                    # Make sure we still have enough data after handling missing values
                    if len(co2_series) >= 14:
                        # Decompose the time series
                        decomposition = seasonal_decompose(co2_series, model='additive', period=7)

                        # Plot trend
                        plt.subplot(4, 2, plot_idx)
                        plt.plot(decomposition.trend)
                        plt.title(f'{location.replace("_", " ").title()} - Trend Component')
                        plt.xlabel('Date')
                        plt.ylabel('CO2 (ppm)')
                        plt.grid(True)

                        # Plot seasonal
                        plt.subplot(4, 2, plot_idx + 1)
                        plt.plot(decomposition.seasonal)
                        plt.title(f'{location.replace("_", " ").title()} - Seasonal Component')
                        plt.xlabel('Date')
                        plt.ylabel('CO2 (ppm)')
                        plt.grid(True)

                        plot_idx += 2
                except Exception as e:
                    print(f"Error in time series decomposition for {location}: {e}")

    plt.tight_layout()
    plt.savefig(f'{output_dir}/time_series_decomposition.png', dpi=300)
    plt.close()

def analyze_moving_averages(datasets, output_dir):
    """Analyze and visualize moving averages of CO2 data."""
    plt.figure(figsize=(15, 15))

    # Plot 1: Daily moving averages
    plt.subplot(3, 1, 1)
    has_data_plot1 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns and not data['CO2'].empty:
            # Original data
            plt.plot(data.index, data['CO2'], alpha=0.3, label=f"{location.replace('_', ' ').title()} - Raw")
            has_data_plot1 = True

            # 24-hour moving average
            daily_ma = data['CO2'].rolling(window=48).mean()  # Assuming 30-minute intervals
            plt.plot(data.index, daily_ma, linewidth=2, label=f"{location.replace('_', ' ').title()} - 24h MA")

    plt.title('Daily Moving Average of CO2 Values')
    plt.xlabel('Date')
    plt.ylabel('CO2 (ppm)')
    if has_data_plot1:
        plt.legend()
    plt.grid(True)

    # Plot 2: Weekly moving averages
    plt.subplot(3, 1, 2)
    has_data_plot2 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns and not data['CO2'].empty:
            # Original data
            plt.plot(data.index, data['CO2'], alpha=0.3, label=f"{location.replace('_', ' ').title()} - Raw")
            has_data_plot2 = True

            # 7-day moving average
            weekly_ma = data['CO2'].rolling(window=336).mean()  # Assuming 30-minute intervals
            plt.plot(data.index, weekly_ma, linewidth=2, label=f"{location.replace('_', ' ').title()} - 7d MA")

    plt.title('Weekly Moving Average of CO2 Values')
    plt.xlabel('Date')
    plt.ylabel('CO2 (ppm)')
    if has_data_plot2:
        plt.legend()
    plt.grid(True)

    # Plot 3: Exponential moving average
    plt.subplot(3, 1, 3)
    has_data_plot3 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns and not data['CO2'].empty:
            # Original data
            plt.plot(data.index, data['CO2'], alpha=0.3, label=f"{location.replace('_', ' ').title()} - Raw")
            has_data_plot3 = True

            # Exponential moving average
            ema = data['CO2'].ewm(span=48).mean()  # 24-hour EMA
            plt.plot(data.index, ema, linewidth=2, label=f"{location.replace('_', ' ').title()} - EMA")

    plt.title('Exponential Moving Average of CO2 Values')
    plt.xlabel('Date')
    plt.ylabel('CO2 (ppm)')
    if has_data_plot3:
        plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/moving_averages.png', dpi=300)
    plt.close()

def analyze_room_comparison(datasets, output_dir):
    """Compare CO2 levels between different rooms."""
    if len(datasets) < 2:
        print("Room comparison requires at least 2 datasets")
        return

    plt.figure(figsize=(15, 15))

    # Plot 1: Average CO2 by hour for each room
    plt.subplot(3, 1, 1)

    for location, data in datasets.items():
        if 'CO2' in data.columns:
            hourly_co2 = data.groupby('hour')['CO2'].mean()
            plt.plot(hourly_co2.index, hourly_co2.values, marker='o', label=location.replace('_', ' ').title())

    plt.title('Average CO2 by Hour of Day - Room Comparison')
    plt.xlabel('Hour of Day')
    plt.ylabel('CO2 (ppm)')
    plt.xticks(range(0, 24, 2))
    plt.legend()
    plt.grid(True)

    # Plot 2: CO2 difference between rooms
    plt.subplot(3, 1, 2)

    locations = list(datasets.keys())
    if len(locations) >= 2:
        # Get common date range
        common_dates = None
        for location, data in datasets.items():
            if 'CO2' in data.columns:
                if common_dates is None:
                    common_dates = set(data.index.date)
                else:
                    common_dates &= set(data.index.date)

        if common_dates:
            # Calculate daily averages
            daily_avgs = {}
            for location, data in datasets.items():
                if 'CO2' in data.columns:
                    daily_avgs[location] = data.resample('D').mean(numeric_only=True)['CO2']

            # Calculate difference between first two locations
            diff = daily_avgs[locations[0]] - daily_avgs[locations[1]]

            # Plot difference
            plt.plot(diff.index, diff.values, marker='o')
            plt.axhline(y=0, color='r', linestyle='-', alpha=0.3)

            plt.title(f'CO2 Difference ({locations[0].replace("_", " ").title()} - {locations[1].replace("_", " ").title()})')
            plt.xlabel('Date')
            plt.ylabel('CO2 Difference (ppm)')
            plt.grid(True)

    # Plot 3: Correlation between rooms
    plt.subplot(3, 1, 3)

    if len(locations) >= 2:
        # Merge datasets on datetime index
        merged_data = None
        for location, data in datasets.items():
            if 'CO2' in data.columns:
                if merged_data is None:
                    merged_data = data[['CO2']].rename(columns={'CO2': f'CO2_{location}'})
                else:
                    merged_data = merged_data.join(data[['CO2']].rename(columns={'CO2': f'CO2_{location}'}), how='outer')

        if merged_data is not None:
            # Calculate correlation matrix
            corr_matrix = merged_data.corr()

            # Plot heatmap
            sns.heatmap(corr_matrix, annot=True, cmap='viridis', vmin=-1, vmax=1)
            plt.title('Correlation Between Room CO2 Levels')

    plt.tight_layout()
    plt.savefig(f'{output_dir}/room_comparison.png', dpi=300)
    plt.close()

def analyze_peak_detection(datasets, output_dir):
    """Detect and analyze peaks in CO2 data."""
    plt.figure(figsize=(15, 15))

    # Plot 1: Peak detection
    plt.subplot(3, 1, 1)
    has_data_plot1 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns:
            co2_data = data['CO2'].dropna()
            if not co2_data.empty and len(co2_data) > 48:  # Need enough data for rolling window
                has_data_plot1 = True

                # Calculate rolling mean and standard deviation
                rolling_mean = co2_data.rolling(window=48).mean()  # 24-hour window
                rolling_std = co2_data.rolling(window=48).std()

                # Define threshold for peaks
                threshold = rolling_mean + 2 * rolling_std

                # Identify peaks
                peaks = co2_data[co2_data > threshold]

                # Plot data and threshold
                plt.plot(co2_data.index, co2_data.values, alpha=0.5, label=f"{location.replace('_', ' ').title()} - Raw")
                plt.plot(rolling_mean.index, rolling_mean.values, label=f"{location.replace('_', ' ').title()} - Mean")
                plt.plot(threshold.index, threshold.values, label=f"{location.replace('_', ' ').title()} - Threshold")

                # Plot peaks
                if not peaks.empty:
                    plt.scatter(peaks.index, peaks.values, color='red', marker='^', label=f"{location.replace('_', ' ').title()} - Peaks")

    plt.title('CO2 Peak Detection')
    plt.xlabel('Date')
    plt.ylabel('CO2 (ppm)')
    if has_data_plot1:
        plt.legend()
    plt.grid(True)

    # Plot 2: Peak frequency by hour of day
    plt.subplot(3, 1, 2)
    has_data_plot2 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns:
            co2_data = data['CO2'].dropna()
            if not co2_data.empty and len(co2_data) > 48:  # Need enough data for rolling window
                # Calculate rolling mean and standard deviation
                rolling_mean = co2_data.rolling(window=48).mean()
                rolling_std = co2_data.rolling(window=48).std()

                # Define threshold for peaks
                threshold = rolling_mean + 2 * rolling_std

                # Identify peaks
                peak_mask = co2_data > threshold
                peak_data = data.loc[peak_mask.index[peak_mask]]

                if not peak_data.empty:
                    # Count peaks by hour
                    peak_counts = peak_data.groupby('hour').size()

                    # Plot peak frequency
                    plt.bar(peak_counts.index, peak_counts.values, alpha=0.7, label=location.replace('_', ' ').title())
                    has_data_plot2 = True

    plt.title('CO2 Peak Frequency by Hour of Day')
    plt.xlabel('Hour of Day')
    plt.ylabel('Number of Peaks')
    plt.xticks(range(0, 24, 2))
    if has_data_plot2:
        plt.legend()
    plt.grid(True)

    # Plot 3: Peak duration analysis
    plt.subplot(3, 1, 3)
    has_data_plot3 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns:
            co2_data = data['CO2'].dropna()
            if not co2_data.empty and len(co2_data) > 48:  # Need enough data for rolling window
                # Calculate rolling mean and standard deviation
                rolling_mean = co2_data.rolling(window=48).mean()
                rolling_std = co2_data.rolling(window=48).std()

                # Define threshold for peaks
                threshold = rolling_mean + 2 * rolling_std

                # Create a series indicating whether each point is a peak
                is_peak = co2_data > threshold

                # Find runs of peaks
                peak_runs = []
                in_peak = False
                start_idx = None

                for i, (idx, val) in enumerate(is_peak.items()):
                    if val and not in_peak:
                        # Start of a peak
                        in_peak = True
                        start_idx = idx
                    elif not val and in_peak:
                        # End of a peak
                        in_peak = False
                        peak_runs.append((start_idx, idx))

                # Calculate peak durations in minutes
                if peak_runs:
                    durations = [(end - start).total_seconds() / 60 for start, end in peak_runs]

                    # Plot histogram of durations
                    plt.hist(durations, bins=20, alpha=0.7, label=location.replace('_', ' ').title())
                    has_data_plot3 = True

    plt.title('CO2 Peak Duration Distribution')
    plt.xlabel('Duration (minutes)')
    plt.ylabel('Frequency')
    if has_data_plot3:
        plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/peak_detection.png', dpi=300)
    plt.close()

def analyze_co2_concentration_heatmap(datasets, output_dir):
    """Create heatmaps of CO2 concentration by hour and day."""
    for location, data in datasets.items():
        if 'CO2' in data.columns:
            plt.figure(figsize=(15, 10))

            # Create a pivot table: rows=day of week, columns=hour of day, values=CO2
            pivot_data = data.pivot_table(
                values='CO2', 
                index='weekday', 
                columns='hour', 
                aggfunc='mean'
            )

            # Reindex to ensure all days and hours are present
            pivot_data = pivot_data.reindex(index=range(7), columns=range(24))

            # Create heatmap
            sns.heatmap(
                pivot_data, 
                cmap='viridis', 
                annot=True, 
                fmt='.0f', 
                linewidths=0.5,
                cbar_kws={'label': 'CO2 (ppm)'}
            )

            # Set labels
            plt.title(f'CO2 Concentration Heatmap - {location.replace("_", " ").title()}')
            plt.xlabel('Hour of Day')
            plt.ylabel('Day of Week')
            plt.yticks(np.arange(7) + 0.5, ['Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday', 'Sunday'])
            plt.xticks(np.arange(0, 24, 3) + 0.5, range(0, 24, 3))

            plt.tight_layout()
            plt.savefig(f'{output_dir}/co2_heatmap_{location}.png', dpi=300)
            plt.close()

    # Create a combined heatmap for month and hour
    plt.figure(figsize=(15, 10))

    for i, (location, data) in enumerate(datasets.items()):
        if 'CO2' in data.columns:
            plt.subplot(len(datasets), 1, i+1)

            # Create a pivot table: rows=month, columns=hour of day, values=CO2
            pivot_data = data.pivot_table(
                values='CO2', 
                index='month', 
                columns='hour', 
                aggfunc='mean'
            )

            # Reindex to ensure all months and hours are present
            pivot_data = pivot_data.reindex(index=range(1, 13), columns=range(24))

            # Create heatmap
            sns.heatmap(
                pivot_data, 
                cmap='viridis', 
                annot=False, 
                linewidths=0.5,
                cbar_kws={'label': 'CO2 (ppm)'}
            )

            # Set labels
            plt.title(f'Monthly CO2 Concentration by Hour - {location.replace("_", " ").title()}')
            plt.xlabel('Hour of Day')
            plt.ylabel('Month')
            plt.yticks(np.arange(12) + 0.5, ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec'])
            plt.xticks(np.arange(0, 24, 3) + 0.5, range(0, 24, 3))

    plt.tight_layout()
    plt.savefig(f'{output_dir}/co2_monthly_heatmap.png', dpi=300)
    plt.close()

def analyze_forecasting_patterns(datasets, output_dir):
    """Identify patterns and potential forecasting models for CO2 data."""
    plt.figure(figsize=(15, 15))

    # Plot 1: Autocorrelation
    plt.subplot(3, 1, 1)
    has_data_plot1 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns and not data['CO2'].empty:
            # Resample to hourly data to reduce noise
            hourly_data = data.resample('h').mean(numeric_only=True)

            # Calculate autocorrelation for each lag separately
            autocorr = []
            for lag in range(1, 49):  # 48 hours
                try:
                    autocorr.append(pd.Series(data['CO2']).autocorr(lag=lag))
                except:
                    autocorr.append(np.nan)

            # Plot autocorrelation
            plt.plot(range(1, 49), autocorr, marker='o', label=location.replace('_', ' ').title())
            has_data_plot1 = True

    plt.title('CO2 Autocorrelation')
    plt.xlabel('Lag (hours)')
    plt.ylabel('Autocorrelation')
    plt.axhline(y=0, color='r', linestyle='-', alpha=0.3)
    if has_data_plot1:
        plt.legend()
    plt.grid(True)

    # Plot 2: Weekly patterns
    plt.subplot(3, 1, 2)
    has_data_plot2 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns and not data['CO2'].empty:
            # Group by day of week and hour
            weekly_pattern = data.groupby(['weekday', 'hour'])['CO2'].mean().unstack()

            # Plot weekly pattern
            for day in range(7):
                if day in weekly_pattern.index:
                    plt.plot(weekly_pattern.columns, weekly_pattern.loc[day], 
                             label=f"{['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun'][day]}" if location == list(datasets.keys())[0] else "")
                    if location == list(datasets.keys())[0]:  # Only count labels from the first location
                        has_data_plot2 = True

    plt.title('Weekly CO2 Patterns')
    plt.xlabel('Hour of Day')
    plt.ylabel('CO2 (ppm)')
    plt.xticks(range(0, 24, 2))
    if has_data_plot2:
        plt.legend()
    plt.grid(True)

    # Plot 3: Periodogram
    plt.subplot(3, 1, 3)
    has_data_plot3 = False

    for location, data in datasets.items():
        if 'CO2' in data.columns and not data['CO2'].empty:
            # Resample to hourly data
            hourly_data = data.resample('h').mean(numeric_only=True)

            # Calculate periodogram
            co2_data = hourly_data['CO2'].dropna().values
            if len(co2_data) > 0:
                f, Pxx = plt.psd(co2_data, NFFT=len(co2_data), Fs=1, label=location.replace('_', ' ').title())
                has_data_plot3 = True

    plt.title('CO2 Power Spectral Density')
    plt.xlabel('Frequency (1/hour)')
    plt.ylabel('Power')
    if has_data_plot3:
        plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/forecasting_patterns.png', dpi=300)
    plt.close()

# Main function to run the analysis
def main():
    # Define data files
    data_files = {
        'bedroom': 'G:\\Study2024-2025\\CO2Analysis\\BedRoomEnv.csv',
        'living_room': 'G:\\Study2024-2025\\CO2Analysis\\LR_env.csv'
    }

    # Define output directory
    output_dir = 'G:\\Study2024-2025\\CO2Analysis\\analysis_results2'

    # Create analyzer
    analyzer = CO2DataAnalyzer(data_files, output_dir)

    # Run analysis
    analyzer.analyze()

    print(f"Analysis complete. Results saved to {output_dir}")

if __name__ == "__main__":
    main()
