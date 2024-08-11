import pandas as pd
import matplotlib.pyplot as plt
import time

# CSVファイルを読み込む
df_picodata = pd.read_csv('/home/koguretaichi/Documents/Flask/PicodataX.csv')

# 'current_time' 列を datetime フォーマットに変換
df_picodata['current_time'] = pd.to_datetime(df_picodata['current_time'])

fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

# Endless loop for reading the data every 30 seconds
while (True):
    # Clearing the current axes in each loop
    ax[0].cla()
    ax[1].cla()

    # Temperature
    ax[0].plot(df_picodata['current_time'], df_picodata['Tempereture'])
    ax[0].set_title('PicoSensor Temperature')
    ax[0].set_ylabel('Temperature')
    ax[0].set_ylim([24, 40])

    # Pressure
    ax[1].plot(df_picodata['current_time'], df_picodata['Pressure'])
    ax[1].set_title('PicoSensor Pressure')
    ax[1].set_xlabel('time')
    ax[1].set_ylabel('Pressure')
    ax[1].set_ylim([970, 1000])

    fig.autofmt_xdate()
    plt.tight_layout(pad=5.0)
    plt.draw()
    plt.pause(0.01)

    # Wait for 30 seconds
    time.sleep(30)

    # Load new data
    df_picodata = pd.read_csv('/home/koguretaichi/Documents/Flask/PicodataX.csv')

    # 'current_time' 列を datetime フォーマットに変換
    df_picodata['current_time'] = pd.to_datetime(df_picodata['current_time'])
