import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns  # Use seaborn for jointplot
from mpl_toolkits.mplot3d import Axes3D  # Import Axes3D for 3D plots

# CSVファイルを読み込む
df = pd.read_csv('Env_data.csv')
df_picodata = pd.read_csv('PicodataX.csv')

# 'current_time' 列を datetime フォーマットに変換
df['current_time'] = pd.to_datetime(df['current_time'])

# Specify the X-axis range
start_date = pd.to_datetime('2024-08-10 18:30:00')
end_date = pd.to_datetime('2024-08-11 23:50:00')

# Create subplots for each column
fig, axs = plt.subplots(5, sharex=True, figsize=(10, 12))  # Graph size changed, and added 2 more plots

# CO2 Value
axs[0].plot(df['current_time'], df['co2_value'], color='blue')  # Changed line graph to bar graph
axs[0].set_title('CO2 Value - Bar Chart')
axs[0].set_xlim([start_date, end_date])
axs[0].set_ylim([400, 800])

# Temperature
axs[1].plot(df['current_time'], df['temperature'], color='red')
axs[1].set_title('Temperature')
axs[1].set_xlim([start_date, end_date])
axs[1].set_ylim([22, 39])

# Pressure
axs[2].plot(df['current_time'], df['pressure'], color='green')
axs[2].set_title('Pressure')
axs[2].set_xlim([start_date, end_date])
axs[2].set_ylim([992, 1005])

# Humidity
axs[3].plot(df['current_time'], df['humidity'], color='purple')
axs[3].set_title('Humidity')
axs[3].set_xlim([start_date, end_date])
axs[3].set_ylim([25, 70])

# Gas Resistance
axs[4].plot(df['current_time'], df['gas_resistance'], color='orange')
axs[4].set_title('Gas Resistance')
axs[4].set_xlim([start_date, end_date])
axs[4].set_ylim([30000, 180000])

fig, axs = plt.subplots(2, figsize=(10, 6))

# Convert 'current_time' to datetime
df_picodata['current_time'] = pd.to_datetime(df_picodata['current_time'])

# Resample the time series data at 1-minute intervals
df_picodata_resampled = df_picodata.resample('1T', on='current_time').mean()

# Make some assumption about the columns of df_picodata for Temperature
axs[0].plot(df_picodata_resampled.index, df_picodata_resampled['Tempereture'], color='blue')
axs[0].set_title('PicoSensor Temperature')
axs[0].set_xlabel('time')
axs[0].set_ylabel('Temperature')
axs[0].set_ylim([27, 42])

# Make some assumption about the columns of df_picodata for Pressure
axs[1].plot(df_picodata_resampled.index, df_picodata_resampled['Pressure'], color='red')
axs[1].set_title('PicoSensor Pressure')
axs[1].set_xlabel('time')
axs[1].set_ylabel('Pressure')
axs[1].set_ylim([970, 1000])

fig.autofmt_xdate()
plt.tight_layout(pad=5.0)
plt.show()

# Create a new jointplot for CO2 Value vs Gas Resistance
sns.jointplot(data=df, x='co2_value', y='gas_resistance', kind='scatter')

# Create another jointplot for Humidity vs Gas Resistance
sns.jointplot(data=df, x='humidity', y='gas_resistance', kind='scatter')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(df['co2_value'], df['gas_resistance'], df['humidity'])
ax.set_xlabel('CO2 Value')
ax.set_ylabel('Gas Resistance')
ax.set_zlabel('Humidity')

# Show the plots
plt.show()

# TerminalからSCPコマンドでラズパイの所定ファイルをPyCharmの作業フォルダに引っ張るコマンド
# scp koguretaichi@raspberrypiG2.local:/home/koguretaichi/Env_data.csv /Users/koguretaichi/PycharmProjects/Study2024-2025
# scp koguretaichi@raspberrypiG.local:/home/koguretaichi/Documents/Flask/PicodataX.csv /Users/koguretaichi/PycharmProjects/Study2024-2025
