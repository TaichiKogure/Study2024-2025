#Lesson17+25

from gpiozero import Button, MCP3008
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from time import sleep
import math
import csv
import threading
from datetime import datetime

#Lesson 25
import os
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')
#ID setting /sys/bus/w1/devices/28-00000084826b で確認
makerobo_DS18b20 = '28-00000084826b'

# Lesson17　GPIO設定
makerobo_DO = Button(17)
makerobo_tempPIN = MCP3008(channel=0)
def makerobo_setup():
    global makerobo_DS18b20
    for i in os.listdir('/sys/bus/w1/devices'):
        if i != 'wl_bus_master1':
            makerobo_DS18b20 = i
def makerobo_read():
    makerobo_location = '/sys/bus/w1/devices/' + makerobo_DS18b20 + '/w1_slave'#センサの場所を保存
    makerobo_tfile = open(makerobo_location)# センサのアドレスを開く
    makerobo_text = makerobo_tfile.read()# 温度のパラメータ取得
    makerobo_tfile.close() #読み取りクローズ
    try:
        secondline = makerobo_text.split('\n')[1]
    except IndexError as e:
        return 0
    temperaturedata = secondline.split(" ")[9]
    tempereture = float(temperaturedata[2:]) / 1000.0
    return tempereture


# グラフのx軸、y軸を保存するリスト設定
x = []
y1 = []
y2 = []
csvfile = "Pasp5EnvData.csv"
csv_writer = csv.writer(open(csvfile, "w"))

csv_writer.writerow(["datetime", "temp_C_sensor1", "temp_C_sensor2"])  # Changed header to datetime


def makerobo_loop(runtime):
    start_time = datetime.now()
    makerobo_Status = 1
    makerobo_tmp = 1

    while True:
        current_datetime = datetime.now()
        if current_datetime - start_time > datetime.timedelta(minutes=runtime):
            makerobo_DO.close()
            makerobo_tempPIN.close()
            break

        makerobo_analogVal = makerobo_tempPIN.value
        makerobo_Vr = float(makerobo_analogVal) * 3.3
        makerobo_Rt = 10000 * makerobo_Vr / (3.3 - makerobo_Vr)
        makerobo_Temp = 1 / (((math.log(makerobo_Rt / 10000)) / 3950) + (1 / (273.15 + 25)))
        temp_c = makerobo_Temp - 273.15
        temp_f = temp_c * 9.0 / 5.0 + 32
        lcd_datetime = current_datetime.strftime("%Y-%m-%d %H:%M:%S")
        print(f"DateTime = {lcd_datetime}, Temp C = {temp_c:.2f}, Temp F = {temp_f:.2f}")

        temperature = makerobo_read()
        # 温度を取得できなかった場合の処理を追加
        if temperature is None:
            continue

        makerobo_tmp = not makerobo_DO.is_pressed
        if makerobo_tmp != makerobo_Status:
          # makerobo_print(makerobo_tmp)
            makerobo_Status = makerobo_tmp


        if makerobo_read() != None:
            print("Current tempereture on Sensor2 : %0.3f C" % makerobo_read())

        # Update the graph
        plt.cla()
        plt.plot(x, y1, label='Temp_C_S1')
        plt.plot(x, y2, label='Temp_C_S2')
        plt.gcf().autofmt_xdate()  # To format x-axis nicely
        date_format = mdates.DateFormatter('%Y-%m-%d %H:%M:%S')  # New addition to format dates on X axis
        plt.gca().xaxis.set_major_formatter(date_format)  # Apply date format to x-axis
        plt.xlabel('DateTime')
        plt.ylabel('Temperature (Celsius)')
        plt.ylim([22, 38])  # Assumes similar range for both sensors
        plt.legend()  # Displays the legend on the plot
        plt.pause(0.3)
        sleep(10)

        x.append(current_datetime)
        y1.append(temp_c)
        y2.append(makerobo_read())
        csv_writer.writerow([lcd_datetime, y1[-1], y2[-1]])  # Write datetime to csv
        
def destroy():
    pass

if __name__ == '__main__':
    try:
        makerobo_setup()
        makerobo_loop(60*24*365)  # XX分後にGPIO解放されます
    except KeyboardInterrupt:
        pass

plt.show()