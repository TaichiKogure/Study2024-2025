import pandas as pd
import matplotlib.pyplot as plt

# CSVファイルを読み込む
# df = pd.read_csv('EnvInfodata01.csv')
df = pd.read_csv('Env_data.csv')

# 'current_time' 列を datetime フォーマットに変換
df['current_time'] = pd.to_datetime(df['current_time'])

# Specify the X-axis range
start_date = pd.to_datetime('2024-08-06 18:30:00')
end_date = pd.to_datetime('2024-08-07 23:50:00')

# Create subplots for each column
fig, axs = plt.subplots(5, sharex=True, figsize=(12, 12))  # Graph size changed

# CO2 Value
axs[0].plot(df['current_time'], df['co2_value'], color='red')  # Color changed
axs[0].set_title('CO2 Value')
axs[0].set_xlim([start_date, end_date])  # X-axis range specified
axs[0].set_ylim([400, 800])  # Y-axis range specified

# Temperature
axs[1].plot(df['current_time'], df['temperature'], color='blue')  # Color changed
axs[1].set_title('Temperature')
axs[1].set_xlim([start_date, end_date])  # X-axis range specified
axs[1].set_ylim([23, 38])  # Y-axis range specified

# Pressure
axs[2].plot(df['current_time'], df['pressure'], color='green')  # Color changed
axs[2].set_title('Pressure')
axs[2].set_xlim([start_date, end_date])  # X-axis range specified
axs[2].set_ylim([995, 1003])  # Y-axis range specified

# Humidity
axs[3].plot(df['current_time'], df['humidity'], color='purple')  # Color changed
axs[3].set_title('Humidity')
axs[3].set_xlim([start_date, end_date])  # X-axis range specified
axs[3].set_ylim([35, 70])  # Y-axis range specified

# Gas Resistance
axs[4].plot(df['current_time'], df['gas_resistance'], color='orange')  # Color changed
axs[4].set_title('Gas Resistance')
axs[4].set_xlim([start_date, end_date])  # X-axis range specified
axs[4].set_ylim([40000, 110000])  # Y-axis range specified

# Optional - Air Quality if column exists
# axs[5].plot(df['current_time'], df['air_quality'], color='pink') # Color changed
# axs[5].set_title('Air Quality')
# axs[5].set_xlim([start_date, end_date])  # X-axis range specified
# axs[5].set_ylim([0, 100]) # Y-axis range specified

# X軸のラベリングを日付形式で
fig.autofmt_xdate()

# Adjust spacing between subplots
plt.tight_layout(pad=5.0)

# Show the plot
plt.show()

# ラズパイからデータファイルを引っ張るときのコマンド
# scp koguretaichi@raspberrypiG2.local:/home/koguretaichi/EnvInfodata01.csv /Users/koguretaichi/PycharmProjects/Study2024-2025

