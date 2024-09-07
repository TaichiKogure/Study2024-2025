import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import time

# Start date
start_date = pd.to_datetime('2024-08-15 18:30:00')

# CSV?????????
df_picodata = pd.read_csv('/home/koguretaichi/Documents/Flask/PicodataX.csv')

# 'current_time' ?? datetime ?????????
df_picodata['current_time'] = pd.to_datetime(df_picodata['current_time'])

fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

# Endless loop for reading the data every 30 seconds
while True:
    # Clear the current axes in each loop
    ax[0].cla()
    ax[1].cla()

    # ???????
    end_date = df_picodata['current_time'].max()

    # Temperature
    ax[0].plot(df_picodata['current_time'], df_picodata['Tempereture'])
    ax[0].set_title('PicoSensor Temperature')
    ax[0].set_ylabel('Temperature')
    ax[0].set_ylim([20, 45])
    ax[0].set_xlim([start_date, end_date])
    ax[0].xaxis.set_major_formatter(mdates.DateFormatter('%Y-%m-%d %H:%M:%S'))

    # Pressure
    ax[1].plot(df_picodata['current_time'], df_picodata['Pressure'])
    ax[1].set_title('PicoSensor Pressure')
    ax[1].set_xlabel('time')
    ax[1].set_ylabel('Pressure')
    ax[1].set_ylim([960, 1020])
    ax[1].set_xlim([start_date, end_date])
    ax[1].xaxis.set_major_formatter(mdates.DateFormatter('%Y-%m-%d %H:%M:%S'))

    fig.autofmt_xdate()
    plt.tight_layout(pad=5.0)
    plt.draw()
    plt.pause(10)

    # Wait for 30 seconds
  #  time.sleep(30)

    # Load new data
    df_picodata = pd.read_csv('/home/koguretaichi/Documents/Flask/PicodataX.csv')

    # 'current_time' ?? datetime ?????????
    df_picodata['current_time'] = pd.to_datetime(df_picodata['current_time'])
