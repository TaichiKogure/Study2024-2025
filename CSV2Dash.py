import dash
from dash import dcc, html
import pandas as pd

# CSV ファイルからデータを読み込む
df = pd.read_csv('Env_data.csv')

app = dash.Dash(__name__)

app.layout = html.Div([
    dcc.Graph(
        id='my-graph',
        figure={
            'data': [
                # 各カラムのデータをプロット
                {'x': df.current_time, 'y': df.co2_value, 'type': 'line', 'name': 'CO2'},
                {'x': df.current_time, 'y': df.temperature, 'type': 'line', 'name': 'Temperature'},
                {'x': df.current_time, 'y': df.pressure, 'type': 'line', 'name': 'Pressure'},
                {'x': df.current_time, 'y': df.humidity, 'type': 'line', 'name': 'Humidity'},
                {'x': df.current_time, 'y': df.gas_resistance, 'type': 'line', 'name': 'Gas Resistance'},
            ],
            'layout': {
                'title': 'Environment Data'
            }
        }
    )
])

if __name__ == '__main__':
    app.run_server(debug=True)
