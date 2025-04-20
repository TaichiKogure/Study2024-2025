# Environmental Data Analysis

This project analyzes environmental data from multiple sensors located in different areas (Bedroom, Living Room, Outside, and Desk) to identify trends in temperature, humidity, CO2 levels, and pressure over time, by season, and by time of day.

## Data Sources

The analysis uses the following CSV files:
- `BedRoomEnv.csv`: Bedroom environment data
- `LR_env.csv`: Living room environment data
- `OutsideEnv.csv`: Outside environment data
- `Env_data_BME680.csv`: Desk environment data (BME680 sensor)

## Features

The analysis includes:

1. **Daily Trends Analysis**
   - Temperature, humidity, CO2, and absolute humidity by hour of day

2. **CO2 Trends Analysis**
   - CO2 levels by hour of day, season, time of day, and weekday/weekend

3. **CO2-Humidity Relationship Analysis**
   - Time series analysis of CO2 and humidity
   - Scatter plots with trend lines showing the relationship between CO2 and humidity

4. **Weekday vs Weekend Patterns**
   - Comparison of temperature, humidity, CO2, pressure, absolute humidity, and gas resistance between weekdays and weekends

5. **Temperature Variations Analysis**
   - Temperature by season, time of day, and month
   - Temperature distribution by season

6. **Humidity Patterns Analysis**
   - Humidity by season, time of day, and month
   - Absolute humidity by season

7. **Extreme Values Analysis**
   - Box plots showing the distribution and extremes of temperature, humidity, CO2, and pressure

8. **Seasonal Trends Analysis**
   - Heatmaps of temperature, humidity, and CO2 by season and time of day
   - Temperature difference between indoor and outdoor by season
   - CO2 and absolute humidity by season and hour

9. **Pressure Trends Analysis**
   - Pressure by hour of day, season, and month
   - Pressure by time of day and season

10. **Monthly Trends Analysis**
    - Monthly trends for temperature, humidity, and CO2
    - Monthly comparison tables for temperature, humidity, CO2, and pressure

11. **Machine Learning Analysis**
    - K-means clustering of temperature data by hour and month
    - K-means clustering of CO2 data by hour and month
    - PCA visualization of clusters

## Seasons Definition

The seasons are defined as follows:
- Spring: April through June
- Summer: July through September
- Fall: October through December
- Winter: January through March

## Time of Day Definition

The time of day is defined as follows:
- Morning: 5:00 AM to 11:59 AM
- Afternoon: 12:00 PM to 4:59 PM
- Evening: 5:00 PM to 9:59 PM
- Night: 10:00 PM to 4:59 AM

## How to Run

1. Ensure you have Python 3.6+ installed
2. Install the required packages:
   ```
   pip install pandas numpy matplotlib seaborn scikit-learn
   ```
3. Place the CSV files in the same directory as the script
4. Run the script:
   ```
   python environmental_data_analysis.py
   ```
5. The analysis results will be saved in the `analysis_results` directory

## Output

The script generates multiple PNG files in the `analysis_results` directory:
- `daily_trends.png`: Daily trends for temperature, humidity, CO2, and absolute humidity
- `co2_trends.png`: CO2 trends by time of day, season, and weekday/weekend
- `co2_humidity_time_series.png`: Time series of CO2 and humidity
- `co2_humidity_relationship.png`: Scatter plots of CO2 vs humidity
- `weekday_weekend_patterns.png`: Comparison of environmental parameters between weekdays and weekends
- `temperature_variations.png`: Temperature variations by season, time of day, and month
- `humidity_patterns.png`: Humidity patterns by season, time of day, and month
- `extreme_values.png`: Box plots of extreme values for temperature, humidity, CO2, and pressure
- `seasonal_trends.png`: Seasonal trends for temperature, humidity, and CO2
- `pressure_trends.png`: Pressure trends by hour, season, and month
- `monthly_trends.png`: Monthly trends for temperature, humidity, and CO2
- `monthly_comparison.png`: Monthly comparison tables for temperature, humidity, CO2, and pressure
- `temperature_ml_*.png`: Machine learning analysis of temperature patterns for each location
- `co2_ml_*.png`: Machine learning analysis of CO2 patterns for each location

## Notes

- The script handles missing data by replacing zeros with NaN values and using appropriate aggregation methods.
- The machine learning analysis uses K-means clustering to identify patterns in temperature and CO2 data.
- The script is designed to work with the specific format of the provided CSV files. If the format changes, the script may need to be updated.