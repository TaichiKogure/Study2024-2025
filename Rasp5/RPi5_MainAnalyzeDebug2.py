import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# CSVファイルを読み込む
bedroom_file_path = "BedRoomEnv.csv"
outside_file_path = "OutsideEnv.csv"

df_bedroom = pd.read_csv(bedroom_file_path, index_col='current_time', parse_dates=True)
df_outside = pd.read_csv(outside_file_path, index_col='current_time', parse_dates=True)

# 欠損値を除去する
df_bedroom = df_bedroom.dropna(subset=['Tempereture', 'Humidity'])
df_outside = df_outside.dropna(subset=['Temperature-outside', 'Humidity-outside'])


# 絶対湿度を計算する関数
def calculate_absolute_humidity(temp, rh):
    # 絶対湿度の計算
    ah = (6.112 * np.exp((17.67 * temp) / (temp + 243.5)) * rh * 2.1674) / (273.15 + temp)
    return ah


# 絶対湿度を新しい列としてデータフレームに追加する
df_bedroom['Absolute_Humidity'] = df_bedroom.apply(
    lambda row: calculate_absolute_humidity(row['Tempereture'], row['Humidity']), axis=1)
df_outside['Absolute_Humidity'] = df_outside.apply(
    lambda row: calculate_absolute_humidity(row['Temperature-outside'], row['Humidity-outside']), axis=1)

# 絶対湿度をプロットする
plt.figure(figsize=(14, 6))
plt.plot(df_bedroom.index.to_numpy(), df_bedroom['Absolute_Humidity'].to_numpy(), label='Absolute Humidity - Bedroom',
         color='b', linestyle='-')
plt.plot(df_outside.index.to_numpy(), df_outside['Absolute_Humidity'].to_numpy(), label='Absolute Humidity - Outside',
         color='r', linestyle='--')
plt.xlabel('Time')
plt.ylabel('Absolute Humidity (g/m³)')
plt.title('Time vs Absolute Humidity (Bedroom & Outside)')
plt.legend()
plt.show()
