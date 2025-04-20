# Environmental Data Dashboard

This interactive dashboard provides a comprehensive viewing system for environmental data collected from multiple sensors located in different areas (Bedroom, Living Room, Outside, and Desk).

## Features

The dashboard includes:

1. **Interactive Time Series Analysis**
   - View temperature, humidity, CO2, pressure, and absolute humidity data over time
   - Filter by date range and location

2. **Daily Patterns Analysis**
   - Visualize how environmental parameters change throughout the day
   - Compare patterns across different locations

3. **Seasonal Patterns Analysis**
   - Analyze how environmental parameters vary by season
   - Compare seasonal patterns across different locations

4. **Monthly Trends Analysis**
   - Track changes in environmental parameters by month
   - Identify long-term trends

5. **Weekday vs Weekend Analysis**
   - Compare environmental patterns between weekdays and weekends
   - Identify differences in occupancy patterns

6. **Statistical Distribution Analysis**
   - View box plots showing the distribution of environmental parameters
   - Compare distributions across different locations

7. **Heatmap Visualization**
   - Visualize environmental parameters by hour of day and day of week
   - Identify patterns in daily and weekly cycles

## How to Run

1. Ensure you have Python 3.6+ installed
2. Install the required packages:
   ```
   pip install dash dash-bootstrap-components pandas numpy plotly
   ```
3. Navigate to the dashboard directory:
   ```
   cd TempAnalysis/dashboard
   ```
4. Run the dashboard:
   ```
   python app.py
   ```
5. Open a web browser and go to:
   ```
   http://127.0.0.1:8050/
   ```

## Using the Dashboard

1. **Date Range Selection**
   - Use the date picker to select the time period you want to analyze

2. **Location Selection**
   - Choose one or more locations (Bedroom, Living Room, Outside, Desk) to compare

3. **Parameter Selection**
   - Select the environmental parameter you want to analyze:
     - Temperature
     - Humidity
     - CO2
     - Pressure
     - Absolute Humidity

4. **Analysis Type Selection**
   - Choose the type of analysis you want to perform:
     - Time Series: View raw data over time
     - Daily Patterns: Analyze patterns by hour of day
     - Seasonal Patterns: Compare data across seasons
     - Monthly Trends: Track changes by month
     - Weekday vs Weekend: Compare weekday and weekend patterns
     - Distribution Analysis: View statistical distributions using box plots

5. **Interactive Graphs**
   - Hover over data points to see detailed information
   - Zoom in/out using the tools in the top-right corner of each graph
   - Download graph images using the camera icon
   - View data summary statistics for each selected location
   - Responsive design adapts to different screen sizes

## Data Sources

The dashboard uses the same data sources as the main analysis script:
- `BedRoomEnv.csv`: Bedroom environment data
- `LR_env.csv`: Living room environment data
- `OutsideEnv.csv`: Outside environment data
- `Env_data_BME680.csv`: Desk environment data (BME680 sensor)

## Notes

- The dashboard automatically handles missing data by replacing zeros with NaN values
- The dashboard uses the same season and time of day definitions as the main analysis script
- For optimal performance, it's recommended to filter the data to a specific time period of interest, especially when working with large datasets
- The dashboard uses the Bootstrap Flatly theme for a modern, clean appearance
- Loading indicators show when data is being processed
