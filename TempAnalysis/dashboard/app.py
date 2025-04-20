import dash
from dash import dcc, html, Input, Output, callback, State
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd
import numpy as np
from datetime import datetime
import os
import traceback
import sys
import dash_bootstrap_components as dbc
from dash.exceptions import PreventUpdate

# Helper functions
def get_season(month):
    """Determine the season based on the month."""
    if month in [12, 1, 2]:
        return 'Winter'
    elif month in [3, 4, 5]:
        return 'Spring'
    elif month in [6, 7, 8]:
        return 'Summer'
    else:  # month in [9, 10, 11]
        return 'Fall'

def get_time_of_day(hour):
    """Determine the time of day based on the hour."""
    if 5 <= hour < 12:
        return 'Morning'
    elif 12 <= hour < 17:
        return 'Afternoon'
    elif 17 <= hour < 22:
        return 'Evening'
    else:  # 22 <= hour < 5
        return 'Night'

def calculate_absolute_humidity(temp, rh):
    """Calculate absolute humidity from temperature and relative humidity."""
    return (6.112 * np.exp((17.67 * temp) / (temp + 243.5)) * rh * 2.1674) / (273.15 + temp)

def load_data(file_path, date_column='current_time'):
    """
    Load and preprocess data from a CSV file.

    Args:
        file_path: Path to the CSV file
        date_column: Name of the column containing datetime information

    Returns:
        Preprocessed pandas DataFrame
    """
    try:
        # Load data
        data = pd.read_csv(file_path)

        # Convert date column to datetime
        data[date_column] = pd.to_datetime(data[date_column])

        # Set date column as index
        data.set_index(date_column, inplace=True)

        # Replace zeros with NaN for better visualization
        for col in data.columns:
            if col.lower().startswith(('temp', 'humidity', 'co2', 'pressure')):
                data[col] = data[col].replace(0, np.nan)

        # Add derived columns
        data['hour'] = data.index.hour
        data['day'] = data.index.day
        data['month'] = data.index.month
        data['year'] = data.index.year
        data['weekday'] = data.index.weekday
        data['is_weekend'] = data['weekday'].apply(lambda x: 1 if x >= 5 else 0)
        data['season'] = data['month'].apply(get_season)
        data['time_of_day'] = data['hour'].apply(get_time_of_day)

        # Rename columns for consistency
        column_mapping = {
            'Tempereture': 'Temperature',
            'Temperature-outside': 'Temperature',
            'Humidity-outside': 'Humidity',
            'Pressure-outside': 'Pressure',
            'GasResistance-outside': 'GasResistance',
            'Temperature_DS18B20': 'Temperature',
            'Humidity_DHT11': 'Humidity',
            'temp_C_sensor1': 'Temperature'
        }

        data.rename(columns=column_mapping, inplace=True, errors='ignore')

        return data

    except Exception as e:
        print(f"Error loading {file_path}: {e}")
        traceback.print_exc()
        return pd.DataFrame()  # Return empty DataFrame on error

def calculate_absolute_humidity_for_location(location, data):
    """Calculate absolute humidity for a specific location."""
    # Define temperature and humidity column names for each location
    temp_humidity_mapping = {
        'bedroom': ('Temperature', 'Humidity'),
        'outside': ('Temperature', 'Humidity'),
        'living_room': ('Temperature', 'Humidity'),
        'desk': ('Temperature', 'Humidity')
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

# Initialize the Dash app with Bootstrap theme
app = dash.Dash(
    __name__, 
    suppress_callback_exceptions=True,
    external_stylesheets=[dbc.themes.FLATLY],
    meta_tags=[
        {"name": "viewport", "content": "width=device-width, initial-scale=1"}
    ]
)

# Define color scheme
COLORS = {
    'bedroom': '#1f77b4',  # Blue
    'outside': '#ff7f0e',  # Orange
    'living_room': '#2ca02c',  # Green
    'desk': '#d62728',  # Red
    'background': '#f9f9f9',
    'card': 'white',
    'text': '#333333'
}

# Define the layout with Bootstrap components
app.layout = dbc.Container([
    dbc.Row([
        dbc.Col([
            html.H1("Environmental Data Dashboard", className="text-center my-4"),
            html.P("Interactive visualization of environmental data from multiple sensors", className="text-center text-muted mb-4")
        ])
    ]),

    dbc.Row([
        # Sidebar with filters
        dbc.Col([
            dbc.Card([
                dbc.CardHeader([
                    html.H4("Filters", className="mb-0"),
                    html.Button(
                        html.I(className="fas fa-sync-alt"), 
                        id="refresh-button",
                        className="btn btn-outline-primary btn-sm float-right",
                        title="Refresh Data"
                    )
                ]),
                dbc.CardBody([
                    html.H6("Date Range"),
                    dcc.DatePickerRange(
                        id='date-range',
                        min_date_allowed=min([data.index.min() for data in datasets.values()]) if datasets else None,
                        max_date_allowed=max([data.index.max() for data in datasets.values()]) if datasets else None,
                        start_date=max([data.index.max() for data in datasets.values()]) - pd.Timedelta(days=7) if datasets else None,
                        end_date=max([data.index.max() for data in datasets.values()]) if datasets else None,
                        className="mb-3 w-100"
                    ),

                    html.H6("Location"),
                    dcc.Dropdown(
                        id='location-dropdown',
                        options=[{'label': loc.replace('_', ' ').title(), 'value': loc} for loc in datasets.keys()],
                        value=list(datasets.keys()),
                        multi=True,
                        className="mb-3"
                    ),

                    html.H6("Parameter"),
                    dcc.Dropdown(
                        id='parameter-dropdown',
                        options=[
                            {'label': 'Temperature', 'value': 'temperature'},
                            {'label': 'Humidity', 'value': 'humidity'},
                            {'label': 'CO2', 'value': 'co2'},
                            {'label': 'Pressure', 'value': 'pressure'},
                            {'label': 'Absolute Humidity', 'value': 'absolute_humidity'}
                        ],
                        value='temperature',
                        className="mb-3"
                    ),

                    html.H6("Analysis Type"),
                    dcc.Dropdown(
                        id='analysis-dropdown',
                        options=[
                            {'label': 'Time Series', 'value': 'time_series'},
                            {'label': 'Daily Patterns', 'value': 'daily_patterns'},
                            {'label': 'Seasonal Patterns', 'value': 'seasonal_patterns'},
                            {'label': 'Monthly Trends', 'value': 'monthly_trends'},
                            {'label': 'Weekday vs Weekend', 'value': 'weekday_weekend'},
                            {'label': 'Distribution Analysis', 'value': 'distribution'}
                        ],
                        value='time_series',
                        className="mb-3"
                    ),

                    # Data summary card
                    html.Div([
                        html.H6("Data Summary", className="mt-4"),
                        html.Div(id="data-summary")
                    ])
                ])
            ], className="shadow-sm mb-4")
        ], md=3),

        # Main content area
        dbc.Col([
            # Main visualization
            dbc.Card([
                dbc.CardHeader(html.H4(id="main-graph-title", className="mb-0")),
                dbc.CardBody([
                    dcc.Loading(
                        dcc.Graph(id='main-graph', config={'displayModeBar': True, 'responsive': True}),
                        type="circle"
                    )
                ])
            ], className="shadow-sm mb-4"),

            # Secondary visualizations
            dbc.Row([
                dbc.Col([
                    dbc.Card([
                        dbc.CardHeader(html.H5(id="secondary-graph-1-title", className="mb-0")),
                        dbc.CardBody([
                            dcc.Loading(
                                dcc.Graph(id='secondary-graph-1', config={'displayModeBar': True, 'responsive': True}),
                                type="circle"
                            )
                        ])
                    ], className="shadow-sm h-100")
                ], md=6),

                dbc.Col([
                    dbc.Card([
                        dbc.CardHeader(html.H5(id="secondary-graph-2-title", className="mb-0")),
                        dbc.CardBody([
                            dcc.Loading(
                                dcc.Graph(id='secondary-graph-2', config={'displayModeBar': True, 'responsive': True}),
                                type="circle"
                            )
                        ])
                    ], className="shadow-sm h-100")
                ], md=6)
            ])
        ], md=9)
    ]),

    # Footer
    dbc.Row([
        dbc.Col([
            html.Hr(),
            html.P("Environmental Data Dashboard © 2025", className="text-center text-muted")
        ])
    ])
], fluid=True, style={"backgroundColor": COLORS['background'], "minHeight": "100vh"})

# Callback to update data summary
@callback(
    Output('data-summary', 'children'),
    [Input('date-range', 'start_date'),
     Input('date-range', 'end_date'),
     Input('location-dropdown', 'value'),
     Input('parameter-dropdown', 'value')]
)
def update_data_summary(start_date, end_date, locations, parameter):
    if not start_date or not end_date or not locations or not parameter:
        return html.P("No data selected")

    start_date = pd.to_datetime(start_date)
    end_date = pd.to_datetime(end_date)

    summary_items = []

    for location in locations:
        if location not in datasets:
            continue

        df = datasets[location]
        df_filtered = df[(df.index >= start_date) & (df.index <= end_date)]

        if df_filtered.empty:
            continue

        param_col = None
        if parameter == 'temperature':
            param_col = 'Temperature'
        elif parameter == 'humidity':
            param_col = 'Humidity'
        elif parameter == 'co2':
            param_col = 'CO2'
        elif parameter == 'pressure':
            param_col = 'Pressure'
        elif parameter == 'absolute_humidity':
            param_col = 'Absolute_Humidity'

        if param_col not in df_filtered.columns:
            continue

        stats = df_filtered[param_col].describe()

        summary_items.append(
            dbc.Card([
                dbc.CardHeader(location.replace('_', ' ').title()),
                dbc.CardBody([
                    html.P(f"Min: {stats['min']:.2f}", className="mb-1"),
                    html.P(f"Max: {stats['max']:.2f}", className="mb-1"),
                    html.P(f"Avg: {stats['mean']:.2f}", className="mb-1"),
                    html.P(f"Records: {len(df_filtered)}", className="mb-1 text-muted")
                ], className="p-2")
            ], className="mb-2")
        )

    if not summary_items:
        return html.P("No data available for the selected criteria")

    return html.Div(summary_items)

# Callback to update graph titles
@callback(
    [Output('main-graph-title', 'children'),
     Output('secondary-graph-1-title', 'children'),
     Output('secondary-graph-2-title', 'children')],
    [Input('parameter-dropdown', 'value'),
     Input('analysis-dropdown', 'value')]
)
def update_graph_titles(parameter, analysis_type):
    parameter_label = {
        'temperature': 'Temperature',
        'humidity': 'Humidity',
        'co2': 'CO2',
        'pressure': 'Pressure',
        'absolute_humidity': 'Absolute Humidity'
    }.get(parameter, parameter.title())

    analysis_label = {
        'time_series': 'Time Series',
        'daily_patterns': 'Daily Patterns',
        'seasonal_patterns': 'Seasonal Patterns',
        'monthly_trends': 'Monthly Trends',
        'weekday_weekend': 'Weekday vs Weekend',
        'distribution': 'Distribution Analysis'
    }.get(analysis_type, analysis_type.replace('_', ' ').title())

    main_title = f"{parameter_label} {analysis_label}"
    secondary_1_title = f"{parameter_label} Statistics"
    secondary_2_title = f"{parameter_label} Heatmap"

    return main_title, secondary_1_title, secondary_2_title

# Define callbacks for the graphs
@callback(
    Output('main-graph', 'figure'),
    [Input('date-range', 'start_date'),
     Input('date-range', 'end_date'),
     Input('location-dropdown', 'value'),
     Input('parameter-dropdown', 'value'),
     Input('analysis-dropdown', 'value')]
)
def update_main_graph(start_date, end_date, locations, parameter, analysis_type):
    if not start_date or not end_date or not locations or not parameter:
        return go.Figure().update_layout(title="No data selected")

    start_date = pd.to_datetime(start_date)
    end_date = pd.to_datetime(end_date)

    # Map parameter to actual column names
    param_mapping = {
        'temperature': 'Temperature',
        'humidity': 'Humidity',
        'co2': 'CO2',
        'pressure': 'Pressure',
        'absolute_humidity': 'Absolute_Humidity'
    }

    param_col = param_mapping.get(parameter)
    if not param_col:
        return go.Figure().update_layout(title="Invalid parameter selected")

    # Time Series Analysis
    if analysis_type == 'time_series':
        fig = go.Figure()

        for location in locations:
            if location not in datasets:
                continue

            df = datasets[location]
            df_filtered = df[(df.index >= start_date) & (df.index <= end_date)]

            if df_filtered.empty or param_col not in df_filtered.columns:
                continue

            fig.add_trace(go.Scatter(
                x=df_filtered.index,
                y=df_filtered[param_col],
                mode='lines',
                name=location.replace('_', ' ').title(),
                line=dict(color=COLORS.get(location, None), width=2),
                hovertemplate='%{y:.2f}<extra>%{x|%Y-%m-%d %H:%M:%S}</extra>'
            ))

        fig.update_layout(
            title=f"{param_col} Time Series",
            xaxis_title="Date",
            yaxis_title=param_col,
            legend_title="Location",
            template="plotly_white",
            hovermode="closest",
            height=500
        )

        return fig

    # Daily Patterns Analysis
    elif analysis_type == 'daily_patterns':
        fig = go.Figure()

        for location in locations:
            if location not in datasets:
                continue

            df = datasets[location]
            df_filtered = df[(df.index >= start_date) & (df.index <= end_date)]

            if df_filtered.empty or param_col not in df_filtered.columns:
                continue

            # Group by hour and calculate mean
            hourly_data = df_filtered.groupby('hour')[param_col].mean()

            fig.add_trace(go.Scatter(
                x=hourly_data.index,
                y=hourly_data.values,
                mode='lines+markers',
                name=location.replace('_', ' ').title(),
                line=dict(color=COLORS.get(location, None), width=2),
                marker=dict(size=8),
                hovertemplate='Hour: %{x}<br>%{y:.2f}<extra></extra>'
            ))

        fig.update_layout(
            title=f"{param_col} Daily Patterns",
            xaxis_title="Hour of Day",
            yaxis_title=f"Average {param_col}",
            legend_title="Location",
            template="plotly_white",
            hovermode="closest",
            height=500,
            xaxis=dict(
                tickmode='linear',
                tick0=0,
                dtick=2,
                ticktext=[f"{h}:00" for h in range(0, 24, 2)],
                tickvals=list(range(0, 24, 2))
            )
        )

        # Add time of day regions
        fig.add_vrect(x0=5, x1=12, fillcolor="rgba(255, 235, 153, 0.2)", layer="below", line_width=0, annotation_text="Morning")
        fig.add_vrect(x0=12, x1=17, fillcolor="rgba(255, 204, 153, 0.2)", layer="below", line_width=0, annotation_text="Afternoon")
        fig.add_vrect(x0=17, x1=22, fillcolor="rgba(204, 204, 255, 0.2)", layer="below", line_width=0, annotation_text="Evening")
        fig.add_vrect(x0=22, x1=24, fillcolor="rgba(153, 153, 204, 0.2)", layer="below", line_width=0, annotation_text="Night")
        fig.add_vrect(x0=0, x1=5, fillcolor="rgba(153, 153, 204, 0.2)", layer="below", line_width=0)

        return fig

    # Seasonal Patterns Analysis
    elif analysis_type == 'seasonal_patterns':
        fig = go.Figure()

        for location in locations:
            if location not in datasets:
                continue

            df = datasets[location]
            df_filtered = df[(df.index >= start_date) & (df.index <= end_date)]

            if df_filtered.empty or param_col not in df_filtered.columns:
                continue

            # Group by season and calculate mean
            seasonal_data = df_filtered.groupby('season')[param_col].mean()

            # Ensure consistent order of seasons
            season_order = ['Winter', 'Spring', 'Summer', 'Fall']
            seasonal_data = seasonal_data.reindex(season_order)

            fig.add_trace(go.Bar(
                x=seasonal_data.index,
                y=seasonal_data.values,
                name=location.replace('_', ' ').title(),
                marker_color=COLORS.get(location, None),
                hovertemplate='Season: %{x}<br>%{y:.2f}<extra></extra>'
            ))

        fig.update_layout(
            title=f"{param_col} Seasonal Patterns",
            xaxis_title="Season",
            yaxis_title=f"Average {param_col}",
            legend_title="Location",
            template="plotly_white",
            hovermode="closest",
            height=500,
            barmode='group'
        )

        return fig

    # Monthly Trends Analysis
    elif analysis_type == 'monthly_trends':
        fig = go.Figure()

        for location in locations:
            if location not in datasets:
                continue

            df = datasets[location]
            df_filtered = df[(df.index >= start_date) & (df.index <= end_date)]

            if df_filtered.empty or param_col not in df_filtered.columns:
                continue

            # Group by month and calculate mean
            monthly_data = df_filtered.groupby(['year', 'month'])[param_col].mean().reset_index()
            monthly_data['date'] = pd.to_datetime(monthly_data[['year', 'month']].assign(day=1))

            fig.add_trace(go.Scatter(
                x=monthly_data['date'],
                y=monthly_data[param_col],
                mode='lines+markers',
                name=location.replace('_', ' ').title(),
                line=dict(color=COLORS.get(location, None), width=2),
                marker=dict(size=8),
                hovertemplate='%{x|%B %Y}<br>%{y:.2f}<extra></extra>'
            ))

        fig.update_layout(
            title=f"{param_col} Monthly Trends",
            xaxis_title="Month",
            yaxis_title=f"Average {param_col}",
            legend_title="Location",
            template="plotly_white",
            hovermode="closest",
            height=500
        )

        return fig

    # Weekday vs Weekend Analysis
    elif analysis_type == 'weekday_weekend':
        fig = go.Figure()

        for location in locations:
            if location not in datasets:
                continue

            df = datasets[location]
            df_filtered = df[(df.index >= start_date) & (df.index <= end_date)]

            if df_filtered.empty or param_col not in df_filtered.columns:
                continue

            # Group by weekday/weekend and hour, then calculate mean
            weekday_data = df_filtered[df_filtered['is_weekend'] == 0].groupby('hour')[param_col].mean()
            weekend_data = df_filtered[df_filtered['is_weekend'] == 1].groupby('hour')[param_col].mean()

            fig.add_trace(go.Scatter(
                x=weekday_data.index,
                y=weekday_data.values,
                mode='lines',
                name=f"{location.replace('_', ' ').title()} - Weekday",
                line=dict(color=COLORS.get(location, None), width=2, dash='solid'),
                hovertemplate='Hour: %{x}<br>%{y:.2f}<extra>Weekday</extra>'
            ))

            fig.add_trace(go.Scatter(
                x=weekend_data.index,
                y=weekend_data.values,
                mode='lines',
                name=f"{location.replace('_', ' ').title()} - Weekend",
                line=dict(color=COLORS.get(location, None), width=2, dash='dot'),
                hovertemplate='Hour: %{x}<br>%{y:.2f}<extra>Weekend</extra>'
            ))

        fig.update_layout(
            title=f"{param_col} Weekday vs Weekend Patterns",
            xaxis_title="Hour of Day",
            yaxis_title=f"Average {param_col}",
            legend_title="Location & Day Type",
            template="plotly_white",
            hovermode="closest",
            height=500,
            xaxis=dict(
                tickmode='linear',
                tick0=0,
                dtick=2,
                ticktext=[f"{h}:00" for h in range(0, 24, 2)],
                tickvals=list(range(0, 24, 2))
            )
        )

        return fig

    # Distribution Analysis
    elif analysis_type == 'distribution':
        fig = go.Figure()

        for location in locations:
            if location not in datasets:
                continue

            df = datasets[location]
            df_filtered = df[(df.index >= start_date) & (df.index <= end_date)]

            if df_filtered.empty or param_col not in df_filtered.columns:
                continue

            fig.add_trace(go.Box(
                y=df_filtered[param_col].dropna(),
                name=location.replace('_', ' ').title(),
                marker_color=COLORS.get(location, None),
                boxmean=True,
                hovertemplate='<b>%{y:.2f}</b><br>Min: %{customdata[0]:.2f}<br>Q1: %{customdata[1]:.2f}<br>Median: %{customdata[2]:.2f}<br>Q3: %{customdata[3]:.2f}<br>Max: %{customdata[4]:.2f}<extra></extra>',
                customdata=np.array([
                    df_filtered[param_col].min(),
                    df_filtered[param_col].quantile(0.25),
                    df_filtered[param_col].median(),
                    df_filtered[param_col].quantile(0.75),
                    df_filtered[param_col].max()
                ]).reshape(1, 5).repeat(len(df_filtered[param_col].dropna()), axis=0)
            ))

        fig.update_layout(
            title=f"{param_col} Distribution Analysis",
            xaxis_title="Location",
            yaxis_title=param_col,
            template="plotly_white",
            hovermode="closest",
            height=500
        )

        return fig

    # Default case
    return go.Figure().update_layout(title="Select an analysis type")

@callback(
    Output('secondary-graph-1', 'figure'),
    [Input('date-range', 'start_date'),
     Input('date-range', 'end_date'),
     Input('location-dropdown', 'value'),
     Input('parameter-dropdown', 'value')]
)
def update_secondary_graph_1(start_date, end_date, locations, parameter):
    if not start_date or not end_date or not locations or not parameter:
        return go.Figure().update_layout(title="No data selected")

    start_date = pd.to_datetime(start_date)
    end_date = pd.to_datetime(end_date)

    # Map parameter to actual column names
    param_mapping = {
        'temperature': 'Temperature',
        'humidity': 'Humidity',
        'co2': 'CO2',
        'pressure': 'Pressure',
        'absolute_humidity': 'Absolute_Humidity'
    }

    param_col = param_mapping.get(parameter)
    if not param_col:
        return go.Figure().update_layout(title="Invalid parameter selected")

    # Create a subplot with 2 rows and 1 column
    fig = make_subplots(rows=2, cols=1, 
                        subplot_titles=(f"{param_col} by Time of Day", f"{param_col} by Day of Week"),
                        vertical_spacing=0.15)

    # First subplot: Parameter by Time of Day
    time_of_day_data = []
    time_of_day_order = ['Morning', 'Afternoon', 'Evening', 'Night']

    for location in locations:
        if location not in datasets:
            continue

        df = datasets[location]
        df_filtered = df[(df.index >= start_date) & (df.index <= end_date)]

        if df_filtered.empty or param_col not in df_filtered.columns:
            continue

        # Group by time of day and calculate mean
        tod_data = df_filtered.groupby('time_of_day')[param_col].mean()
        # Ensure consistent order
        tod_data = tod_data.reindex(time_of_day_order)

        fig.add_trace(
            go.Bar(
                x=tod_data.index,
                y=tod_data.values,
                name=location.replace('_', ' ').title(),
                marker_color=COLORS.get(location, None),
                hovertemplate='%{y:.2f}<extra>%{x}</extra>'
            ),
            row=1, col=1
        )

    # Second subplot: Parameter by Day of Week
    day_names = ['Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday', 'Sunday']

    for location in locations:
        if location not in datasets:
            continue

        df = datasets[location]
        df_filtered = df[(df.index >= start_date) & (df.index <= end_date)]

        if df_filtered.empty or param_col not in df_filtered.columns:
            continue

        # Group by weekday and calculate mean
        weekday_data = df_filtered.groupby('weekday')[param_col].mean()

        fig.add_trace(
            go.Bar(
                x=[day_names[i] for i in weekday_data.index],
                y=weekday_data.values,
                name=location.replace('_', ' ').title(),
                marker_color=COLORS.get(location, None),
                showlegend=False,  # Don't repeat legend entries
                hovertemplate='%{y:.2f}<extra>%{x}</extra>'
            ),
            row=2, col=1
        )

    # Update layout
    fig.update_layout(
        height=600,
        template="plotly_white",
        legend_title="Location",
        barmode='group'
    )

    # Update y-axis labels
    fig.update_yaxes(title_text=f"Average {param_col}", row=1, col=1)
    fig.update_yaxes(title_text=f"Average {param_col}", row=2, col=1)

    return fig

@callback(
    Output('secondary-graph-2', 'figure'),
    [Input('date-range', 'start_date'),
     Input('date-range', 'end_date'),
     Input('location-dropdown', 'value'),
     Input('parameter-dropdown', 'value')]
)
def update_secondary_graph_2(start_date, end_date, locations, parameter):
    if not start_date or not end_date or not locations or not parameter:
        return go.Figure().update_layout(title="No data selected")

    start_date = pd.to_datetime(start_date)
    end_date = pd.to_datetime(end_date)

    # Map parameter to actual column names
    param_mapping = {
        'temperature': 'Temperature',
        'humidity': 'Humidity',
        'co2': 'CO2',
        'pressure': 'Pressure',
        'absolute_humidity': 'Absolute_Humidity'
    }

    param_col = param_mapping.get(parameter)
    if not param_col:
        return go.Figure().update_layout(title="Invalid parameter selected")

    # If multiple locations are selected, use the first one for the heatmap
    if not locations:
        return go.Figure().update_layout(title="No location selected")

    location = locations[0]
    if location not in datasets:
        return go.Figure().update_layout(title=f"No data for {location}")

    df = datasets[location]
    df_filtered = df[(df.index >= start_date) & (df.index <= end_date)]

    if df_filtered.empty or param_col not in df_filtered.columns:
        return go.Figure().update_layout(title=f"No {param_col} data for {location}")

    # Create a pivot table: hour of day vs day of week
    pivot_data = df_filtered.pivot_table(
        values=param_col,
        index='hour',
        columns='weekday',
        aggfunc='mean'
    )

    # Rename columns to day names
    day_names = ['Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday', 'Sunday']
    pivot_data.columns = [day_names[i] for i in pivot_data.columns]

    # Create heatmap
    fig = go.Figure(data=go.Heatmap(
        z=pivot_data.values,
        x=pivot_data.columns,
        y=[f"{h:02d}:00" for h in pivot_data.index],
        colorscale='Viridis',
        hovertemplate='Day: %{x}<br>Hour: %{y}<br>Value: %{z:.2f}<extra></extra>'
    ))

    # Update layout
    fig.update_layout(
        title=f"{param_col} Heatmap for {location.replace('_', ' ').title()}",
        xaxis_title="Day of Week",
        yaxis_title="Hour of Day",
        height=600,
        template="plotly_white",
        yaxis=dict(
            tickmode='array',
            tickvals=[f"{h:02d}:00" for h in range(0, 24, 3)],
            autorange="reversed"  # To have 00:00 at the top
        )
    )

    # Add color bar title
    fig.update_coloraxes(colorbar_title=param_col)

    return fig

# Run the app
if __name__ == '__main__':
    app.run_server(debug=True)
