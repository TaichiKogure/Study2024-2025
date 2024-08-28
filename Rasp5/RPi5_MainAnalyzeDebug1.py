
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# CSVファイルを読み込む
file_path = "BedRoomEnv.csv"
df = pd.read_csv(file_path, index_col='current_time', parse_dates=True)

# 欠損値を除去する
df = df.dropna(subset=['Tempereture', 'Humidity'])

# 絶対湿度を計算する関数
def calculate_absolute_humidity(temp, rh):
    # 絶対湿度の計算
    ah = (6.112 * np.exp((17.67 * temp) / (temp + 243.5)) * rh * 2.1674) / (273.15 + temp)
    return ah

# 絶対湿度を新しい列としてデータフレームに追加する
df['Absolute_Humidity'] = df.apply(lambda row: calculate_absolute_humidity(row['Tempereture'], row['Humidity']), axis=1)

# 絶対湿度をプロットする
plt.figure(figsize=(14, 6))
plt.plot(df.index.to_numpy(), df['Absolute_Humidity'].to_numpy(), label='Absolute Humidity', color='b', linestyle='-')
plt.xlabel('Time')
plt.ylabel('Absolute Humidity (g/m³)')
plt.title('Time vs Absolute Humidity')
plt.legend()
plt.show()