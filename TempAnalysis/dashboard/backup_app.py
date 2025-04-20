import dash
from dash import dcc, html, Input, Output, callback
import plotly.express as px
import plotly.graph_objects as go
import pandas as pd
import numpy as np
from datetime import datetime
import os
import traceback
import sys

# Define constants
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

# Helper functions
def get_season(month):
    try:
        if not isinstance(month, (int, float)) or pd.isna(month):
            return 'Unknown'
        month_int = int(month)
        if month_int < 1 or month_int > 12:
            return 'Unknown'
        return next((season for season, months in SEASONS.items() if month_int in months), 'Unknown')
    except (ValueError, TypeError):
        return 'Unknown'

def get_time_of_day(hour):
    try:
        if not isinstance(hour, (int, float)) or pd.isna(hour):
            return 'Unknown'
        hour_int = int(hour)
        if hour_int < 0 or hour_int > 23:
            return 'Unknown'
        return next((period for period, hours in TIME_OF_DAY.items() if hour_int in hours), 'Unknown')
    except (ValueError, TypeError, StopIteration):
        return 'Unknown'

def calculate_absolute_humidity(temp, rh):
    return (6.112 * np.exp((17.67 * temp) / (temp + 243.5)) * rh * 2.1674) / (273.15 + temp)

def load_data(file_path, date_column='current_time'):
    print(f"Loading data from {file_path}...")

    # Try to read the CSV file with explicit error handling
    data = pd.read_csv(file_path)

    # Check if the date column exists in the data
    if date_column not in data.columns:
        raise ValueError(f"Date column '{date_column}' not found in the CSV file. Available columns: {', '.join(data.columns)}")

    # Check for empty or all-NaN date column
    if data[date_column].isna().all():
        raise ValueError(f"Date column '{date_column}' contains only NaN values")

    print(f"Sample date values: {data[date_column].head().tolist()}")

    # Convert date column to datetime with error handling for different formats
    try:
        # First try with explicit format to avoid deprecation warning
        data[date_column] = pd.to_datetime(data[date_column], format='%Y-%m-%d %H:%M:%S')
        print(f"Successfully parsed dates with format '%Y-%m-%d %H:%M:%S'")
    except ValueError as e1:
        print(f"Failed to parse dates with format '%Y-%m-%d %H:%M:%S': {e1}")
        try:
            # If that fails, try with a different format
            data[date_column] = pd.to_datetime(data[date_column], format='%Y-%m-%d %H:%M:%S.%f')
            print(f"Successfully parsed dates with format '%Y-%m-%d %H:%M:%S.%f'")
        except ValueError as e2:
            print(f"Failed to parse dates with format '%Y-%m-%d %H:%M:%S.%f': {e2}")
            # If all explicit formats fail, try some additional common formats
            try:
                data[date_column] = pd.to_datetime(data[date_column], format='%Y/%m/%d %H:%M:%S')
                print(f"Successfully parsed dates with format '%Y/%m/%d %H:%M:%S'")
            except ValueError as e3:
                print(f"Failed to parse dates with format '%Y/%m/%d %H:%M:%S': {e3}")
                # If all explicit formats fail, fall back to the default parser with a warning
                print(f"Warning: Could not parse dates with explicit format for {file_path}. Using default parser.")
                data[date_column] = pd.to_datetime(data[date_column], errors='coerce')

                # Check if any dates were successfully parsed
                if data[date_column].isna().all():
                    raise ValueError("All dates were converted to NaT (Not a Time). Check the date format in your CSV file.")

    # Set date column as index
    data.set_index(date_column, inplace=True)

    # Add time-related features
    data['month'] = data.index.month
    data['day'] = data.index.day
    data['hour'] = data.index.hour
    data['weekday'] = data.index.weekday
    data['is_weekend'] = data['weekday'].apply(lambda x: 1 if x >= 5 else 0)

    # Add season and time of day
    data['season'] = data['month'].apply(get_season)
    data['time_of_day'] = data['hour'].apply(get_time_of_day)

    # Handle missing values
    data = data.replace(0, np.nan)

    print(f"Data loaded. Shape: {data.shape}")
    return data

def calculate_absolute_humidity_for_location(location, data):
    temp_humidity_mapping = {
        'bedroom': ('Tempereture', 'Humidity'),
        'outside': ('Temperature-outside', 'Humidity-outside'),
        'living_room': ('Temperature_DS18B20', 'Humidity_DHT11'),
        'desk': ('temperature', 'humidity')
    }

    if location in temp_humidity_mapping:
        temp_col, hum_col = temp_humidity_mapping[location]
        if temp_col in data.columns and hum_col in data.columns:
            data['Absolute_Humidity'] = calculate_absolute_humidity(
                data[temp_col].fillna(0),
                data[hum_col].fillna(0)
            )
    return data

# Load data
# Use absolute paths to ensure files are found correctly
base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
data_files = {
    'bedroom': os.path.join(base_dir, 'BedRoomEnv.csv'),
    'outside': os.path.join(base_dir, 'OutsideEnv.csv'),
    'living_room': os.path.join(base_dir, 'LR_env.csv'),
    'desk': os.path.join(base_dir, 'Env_data_BME680.csv')
}

datasets = {}
for location, file_path in data_files.items():
    try:
        # Check if file exists before attempting to load it
        if not os.path.exists(file_path):
            print(f"Error: File not found: {file_path}")
            continue

        data = load_data(file_path)
        data = calculate_absolute_humidity_for_location(location, data)
        datasets[location] = data
    except Exception as e:
        print(f"Error loading {file_path}: {e}")
        print("Detailed error information:")
        traceback.print_exc()

# Check if any datasets were loaded successfully
if not datasets:
    print("Error: No datasets were loaded successfully. Please check the file paths and formats.")
    sys.exit(1)

# Initialize the Dash app
app = dash.Dash(__name__, suppress_callback_exceptions=True)

# Define the layout
app.layout = html.Div([
    html.H1("Environmental Data Dashboard"),

    html.Div([
        html.Div([
            html.H3("Filters"),
            html.Label("Date Range:"),
            dcc.DatePickerRange(
                id='date-range',
                min_date_allowed=min([data.index.min() for data in datasets.values()]) if datasets else None,
                max_date_allowed=max([data.index.max() for data in datasets.values()]) if datasets else None,
                start_date=min([data.index.min() for data in datasets.values()]) if datasets else None,
                end_date=max([data.index.max() for data in datasets.values()]) if datasets else None,
            ),
            html.Br(),
            html.Label("Location:"),
            dcc.Dropdown(
                id='location-dropdown',
                options=[{'label': loc.replace('_', ' ').title(), 'value': loc} for loc in datasets.keys()],
                value=list(datasets.keys()),
                multi=True
            ),
            html.Br(),
            html.Label("Parameter:"),
            dcc.Dropdown(
                id='parameter-dropdown',
                options=[
                    {'label': 'Temperature', 'value': 'temperature'},
                    {'label': 'Humidity', 'value': 'humidity'},
                    {'label': 'CO2', 'value': 'co2'},
                    {'label': 'Pressure', 'value': 'pressure'},
                    {'label': 'Absolute Humidity', 'value': 'absolute_humidity'}
                ],
                value='temperature'
            ),
            html.Br(),
            html.Label("Analysis Type:"),
            dcc.Dropdown(
                id='analysis-dropdown',
                options=[
                    {'label': 'Time Series', 'value': 'time_series'},
                    {'label': 'Daily Patterns', 'value': 'daily_patterns'},
                    {'label': 'Seasonal Patterns', 'value': 'seasonal_patterns'},
                    {'label': 'Monthly Trends', 'value': 'monthly_trends'},
                    {'label': 'Weekday vs Weekend', 'value': 'weekday_weekend'}
                ],
                value='time_series'
            )
        ], style={'width': '25%', 'display': 'inline-block', 'vertical-align': 'top', 'padding': '20px'}),

        html.Div([
            dcc.Graph(id='main-graph')
        ], style={'width': '75%', 'display': 'inline-block', 'padding': '20px'})
    ]),

    html.Div([
        html.Div([
            dcc.Graph(id='secondary-graph-1')
        ], style={'width': '50%', 'display': 'inline-block'}),

        html.Div([
            dcc.Graph(id='secondary-graph-2')
        ], style={'width': '50%', 'display': 'inline-block'})
    ])
])

# Define callbacks
@callback(
    Output('main-graph', 'figure'),
    [Input('date-range', 'start_date'),
     Input('date-range', 'end_date'),
     Input('location-dropdown', 'value'),
     Input('parameter-dropdown', 'value'),
     Input('analysis-dropdown', 'value')]
)
def update_main_graph(start_date, end_date, locations, parameter, analysis_type):
    if not isinstance(locations, list):
        locations = [locations]

    # Parameter to column mapping
    param_mapping = {
        'temperature': {
            'bedroom': 'Tempereture',
            'outside': 'Temperature-outside',
            'living_room': 'Temperature_DS18B20',
            'desk': 'temperature'
        },
        'humidity': {
            'bedroom': 'Humidity',
            'outside': 'Humidity-outside',
            'living_room': 'Humidity_DHT11',
            'desk': 'humidity'
        },
        'co2': {
            'bedroom': 'CO2',
            'living_room': 'CO2'
        },
        'pressure': {
            'bedroom': 'Pressure',
            'outside': 'Pressure-outside',
            'desk': 'pressure'
        },
        'absolute_humidity': {
            'bedroom': 'Absolute_Humidity',
            'outside': 'Absolute_Humidity',
            'living_room': 'Absolute_Humidity',
            'desk': 'Absolute_Humidity'
        }
    }

    # Parameter labels
    param_labels = {
        'temperature': 'Temperature (°C)',
        'humidity': 'Humidity (%)',
        'co2': 'CO2 (ppm)',
        'pressure': 'Pressure (hPa)',
        'absolute_humidity': 'Absolute Humidity (g/m³)'
    }

    # Filter data by date range
    filtered_datasets = {}
    for loc in locations:
        if loc in datasets:
            filtered_data = datasets[loc].copy()
            filtered_data = filtered_data.loc[start_date:end_date]
            filtered_datasets[loc] = filtered_data

    # Create figure based on analysis type
    if analysis_type == 'time_series':
        fig = go.Figure()

        for loc in locations:
            if loc in filtered_datasets and loc in param_mapping[parameter]:
                col = param_mapping[parameter][loc]
                if col in filtered_datasets[loc].columns:
                    fig.add_trace(go.Scatter(
                        x=filtered_datasets[loc].index,
                        y=filtered_datasets[loc][col],
                        mode='lines',
                        name=loc.replace('_', ' ').title()
                    ))

        fig.update_layout(
            title=f"{parameter.title()} Time Series",
            xaxis_title="Date",
            yaxis_title=param_labels[parameter],
            legend_title="Location",
            template="plotly_white"
        )

    elif analysis_type == 'daily_patterns':
        fig = go.Figure()

        for loc in locations:
            if loc in filtered_datasets and loc in param_mapping[parameter]:
                col = param_mapping[parameter][loc]
                if col in filtered_datasets[loc].columns:
                    hourly_data = filtered_datasets[loc].groupby('hour')[col].mean()
                    fig.add_trace(go.Scatter(
                        x=hourly_data.index,
                        y=hourly_data.values,
                        mode='lines+markers',
                        name=loc.replace('_', ' ').title()
                    ))

        fig.update_layout(
            title=f"Average {parameter.title()} by Hour of Day",
            xaxis_title="Hour of Day",
            yaxis_title=param_labels[parameter],
            legend_title="Location",
            template="plotly_white",
            xaxis=dict(tickmode='array', tickvals=list(range(0, 24, 2)))
        )

    elif analysis_type == 'seasonal_patterns':
        fig = go.Figure()

        for loc in locations:
            if loc in filtered_datasets and loc in param_mapping[parameter]:
                col = param_mapping[parameter][loc]
                if col in filtered_datasets[loc].columns:
                    seasonal_data = filtered_datasets[loc].groupby('season')[col].mean()
                    # Reorder seasons
                    season_order = ['Winter', 'Spring', 'Summer', 'Fall']
                    seasonal_data = seasonal_data.reindex(season_order)

                    fig.add_trace(go.Bar(
                        x=seasonal_data.index,
                        y=seasonal_data.values,
                        name=loc.replace('_', ' ').title()
                    ))

        fig.update_layout(
            title=f"Average {parameter.title()} by Season",
            xaxis_title="Season",
            yaxis_title=param_labels[parameter],
            legend_title="Location",
            template="plotly_white",
            barmode='group'
        )

    elif analysis_type == 'monthly_trends':
        fig = go.Figure()

        for loc in locations:
            if loc in filtered_datasets and loc in param_mapping[parameter]:
                col = param_mapping[parameter][loc]
                if col in filtered_datasets[loc].columns:
                    monthly_data = filtered_datasets[loc].groupby('month')[col].mean()

                    fig.add_trace(go.Scatter(
                        x=monthly_data.index,
                        y=monthly_data.values,
                        mode='lines+markers',
                        name=loc.replace('_', ' ').title()
                    ))

        fig.update_layout(
            title=f"Monthly {parameter.title()} Trends",
            xaxis_title="Month",
            yaxis_title=param_labels[parameter],
            legend_title="Location",
            template="plotly_white",
            xaxis=dict(tickmode='array', tickvals=list(range(1, 13)))
        )

    elif analysis_type == 'weekday_weekend':
        fig = go.Figure()

        for loc in locations:
            if loc in filtered_datasets and loc in param_mapping[parameter]:
                col = param_mapping[parameter][loc]
                if col in filtered_datasets[loc].columns:
                    # Weekday
                    weekday_data = filtered_datasets[loc][filtered_datasets[loc]['is_weekend'] == 0].groupby('hour')[col].mean()
                    fig.add_trace(go.Scatter(
                        x=weekday_data.index,
                        y=weekday_data.values,
                        mode='lines',
                        name=f"{loc.replace('_', ' ').title()} - Weekday"
                    ))

                    # Weekend
                    weekend_data = filtered_datasets[loc][filtered_datasets[loc]['is_weekend'] == 1].groupby('hour')[col].mean()
                    fig.add_trace(go.Scatter(
                        x=weekend_data.index,
                        y=weekend_data.values,
                        mode='lines',
                        line=dict(dash='dash'),
                        name=f"{loc.replace('_', ' ').title()} - Weekend"
                    ))

        fig.update_layout(
            title=f"{parameter.title()} by Hour - Weekday vs Weekend",
            xaxis_title="Hour of Day",
            yaxis_title=param_labels[parameter],
            legend_title="Location",
            template="plotly_white",
            xaxis=dict(tickmode='array', tickvals=list(range(0, 24, 2)))
        )

    return fig

@callback(
    Output('secondary-graph-1', 'figure'),
    [Input('date-range', 'start_date'),
     Input('date-range', 'end_date'),
     Input('location-dropdown', 'value'),
     Input('parameter-dropdown', 'value')]
)
def update_secondary_graph_1(start_date, end_date, locations, parameter):
    if not isinstance(locations, list):
        locations = [locations]

    # Parameter to column mapping (same as in update_main_graph)
    param_mapping = {
        'temperature': {
            'bedroom': 'Tempereture',
            'outside': 'Temperature-outside',
            'living_room': 'Temperature_DS18B20',
            'desk': 'temperature'
        },
        'humidity': {
            'bedroom': 'Humidity',
            'outside': 'Humidity-outside',
            'living_room': 'Humidity_DHT11',
            'desk': 'humidity'
        },
        'co2': {
            'bedroom': 'CO2',
            'living_room': 'CO2'
        },
        'pressure': {
            'bedroom': 'Pressure',
            'outside': 'Pressure-outside',
            'desk': 'pressure'
        },
        'absolute_humidity': {
            'bedroom': 'Absolute_Humidity',
            'outside': 'Absolute_Humidity',
            'living_room': 'Absolute_Humidity',
            'desk': 'Absolute_Humidity'
        }
    }

    # Parameter labels
    param_labels = {
        'temperature': 'Temperature (°C)',
        'humidity': 'Humidity (%)',
        'co2': 'CO2 (ppm)',
        'pressure': 'Pressure (hPa)',
        'absolute_humidity': 'Absolute Humidity (g/m³)'
    }

    # Filter data by date range
    filtered_datasets = {}
    for loc in locations:
        if loc in datasets:
            filtered_data = datasets[loc].copy()
            filtered_data = filtered_data.loc[start_date:end_date]
            filtered_datasets[loc] = filtered_data

    # Create box plot of parameter distribution by location
    fig = go.Figure()

    for loc in locations:
        if loc in filtered_datasets and loc in param_mapping[parameter]:
            col = param_mapping[parameter][loc]
            if col in filtered_datasets[loc].columns:
                fig.add_trace(go.Box(
                    y=filtered_datasets[loc][col].dropna(),
                    name=loc.replace('_', ' ').title()
                ))

    fig.update_layout(
        title=f"{parameter.title()} Distribution by Location",
        yaxis_title=param_labels[parameter],
        template="plotly_white"
    )

    return fig

@callback(
    Output('secondary-graph-2', 'figure'),
    [Input('date-range', 'start_date'),
     Input('date-range', 'end_date'),
     Input('location-dropdown', 'value'),
     Input('parameter-dropdown', 'value')]
)
def update_secondary_graph_2(start_date, end_date, locations, parameter):
    if not isinstance(locations, list):
        locations = [locations]

    # Parameter to column mapping (same as in update_main_graph)
    param_mapping = {
        'temperature': {
            'bedroom': 'Tempereture',
            'outside': 'Temperature-outside',
            'living_room': 'Temperature_DS18B20',
            'desk': 'temperature'
        },
        'humidity': {
            'bedroom': 'Humidity',
            'outside': 'Humidity-outside',
            'living_room': 'Humidity_DHT11',
            'desk': 'humidity'
        },
        'co2': {
            'bedroom': 'CO2',
            'living_room': 'CO2'
        },
        'pressure': {
            'bedroom': 'Pressure',
            'outside': 'Pressure-outside',
            'desk': 'pressure'
        },
        'absolute_humidity': {
            'bedroom': 'Absolute_Humidity',
            'outside': 'Absolute_Humidity',
            'living_room': 'Absolute_Humidity',
            'desk': 'Absolute_Humidity'
        }
    }

    # Parameter labels
    param_labels = {
        'temperature': 'Temperature (°C)',
        'humidity': 'Humidity (%)',
        'co2': 'CO2 (ppm)',
        'pressure': 'Pressure (hPa)',
        'absolute_humidity': 'Absolute Humidity (g/m³)'
    }

    # Filter data by date range
    filtered_datasets = {}
    for loc in locations:
        if loc in datasets:
            filtered_data = datasets[loc].copy()
            filtered_data = filtered_data.loc[start_date:end_date]
            filtered_datasets[loc] = filtered_data

    # Create heatmap of parameter by time of day and day of week
    fig = go.Figure()

    # Use the first selected location for the heatmap
    if locations and locations[0] in filtered_datasets and locations[0] in param_mapping[parameter]:
        loc = locations[0]
        col = param_mapping[parameter][loc]

        if col in filtered_datasets[loc].columns:
            # Create pivot table
            pivot_data = filtered_datasets[loc].pivot_table(
                values=col,
                index='hour',
                columns='weekday',
                aggfunc='mean'
            )

            # Create heatmap
            fig = px.imshow(
                pivot_data,
                labels=dict(x="Day of Week", y="Hour of Day", color=param_labels[parameter]),
                x=['Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday', 'Sunday'],
                y=list(range(24)),
                aspect="auto",
                title=f"{loc.replace('_', ' ').title()} {parameter.title()} by Hour and Day of Week"
            )

    return fig

# Run the app
if __name__ == '__main__':
    app.run(debug=True)