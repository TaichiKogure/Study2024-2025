#Lesson17+25

from gpiozero import Button, MCP3008
import matplotlib.pyplot as plt
from time import sleep, time
import math
import csv
import threading

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
    makerobo_location = '/sys/bus/w1/devices/' + makerobo_DS18b20 + '/wl_slave'#センサの場所を保存
    makerobo_tfile = open(makerobo_location)# センサのアドレスを開く
    makerobo_text = makerobo_tfile.read()# 温度のパラメータ取得
    makerobo_tfile.close() #読み取りクローズ
    try:
        secondline = makerobo_text.split('\n')[1]
    except IndexError as e:
        return 0
    temperaturedata = secondline.split("")[9]
    tempereture = float(temperaturedata[2:]) / 1000.0
    return tempereture


# グラフのx軸、y軸を保存するリスト設定
x = []
y1 = []
y2 = []

# csvファイル名設定
csvfile = "temp_data.csv"
csv_writer = csv.writer(open(csvfile, "w"))
csv_writer.writerow(["time(min)", "temp_C_sensor1", "temp_C_sensor2"])


def makerobo_print(x):
    if x == 1:
        print('Better')
    if x == 0:
        print('Too Hot !!')

def makerobo_loop(runtime):
    start_time = time()
    makerobo_Status = 1
    makerobo_tmp = 1

    while True:
        current_time = time()
        if current_time - start_time > runtime * 60:  # runtime分経過後、GPIO解放
            makerobo_DO.close()
            makerobo_tempPIN.close()
            break

        makerobo_analogVal = makerobo_tempPIN.value
        makerobo_Vr = float(makerobo_analogVal) * 3.3
        makerobo_Rt = 10000 * makerobo_Vr / (3.3 - makerobo_Vr)
        makerobo_Temp = 1 / (((math.log(makerobo_Rt / 10000)) / 3950) + (1 / (273.15 + 25)))
        temp_c = makerobo_Temp - 273.15
        temp_f = temp_c * 9.0 / 5.0 + 32
        print(f"Temp C = {temp_c:.2f}, Temp F = {temp_f:.2f}")

        temperature = makerobo_read()
        # 温度を取得できなかった場合の処理を追加
        if temperature is None:
            continue

        x.append(current_time - start_time)
        y1.append(temp_c)
        y2.append(makerobo_read())
        csv_writer.writerow([x[-1], y1[-1], y2[-1]])  # csvに書き出し

        makerobo_tmp = not makerobo_DO.is_pressed
        if makerobo_tmp != makerobo_Status:
            makerobo_print(makerobo_tmp)
            makerobo_Status = makerobo_tmp


        if makerobo_read() != None:
            print("Current tempereture on Sensor2 : %0.3f C" % makerobo_read())

        # Update the graph
        plt.cla()  # これがないとLabelが増え続けてグラフが埋まるのでつける。
        plt.plot(x, y1, label='Temp_C_S1')
        plt.plot(x, y2, label='Temp_C_S2')
        plt.xlabel('Time (min)')
        plt.ylabel('Temperature (Celsius)')
        plt.ylim([24, 43])  # Assumes similar range for both sensors
        plt.legend()  # Displays the legend on the plot
        plt.pause(0.05)
        sleep(10)

def destroy():
    pass

if __name__ == '__main__':
    try:
        makerobo_setup()
        makerobo_loop(1440)  # 10分後にGPIO解放されます
    except KeyboardInterrupt:
        pass

plt.show()

#%%

################################################
################################################
################## OLD ver #####################
################################################
################################################

from gpiozero import Button, MCP3008
import matplotlib.pyplot as plt
from time import sleep, time
import math
import csv

# GPIO settings
makerobo_DO = Button(17)
makerobo_tempPIN = MCP3008(channel=0)

# Lists to store x and y values for the graph
x = []
y = []

# CSV filename
csvfile = "temp_data.csv"
csv_writer = csv.writer(open(csvfile, "w"))
csv_writer.writerow(["time(min)", "temp_C"])


def makerobo_print(x):
    if x == 1:
        print('Better')
    if x == 0:
        print('Too Hot !!')


def makerobo_loop(runtime):
    start_time = time()
    makerobo_Status = 1
    makerobo_tmp = 1

    while True:
        current_time = time()
        if current_time - start_time > runtime * 60:  # After runtime minutes, GPIO is released.
            makerobo_DO.close()
            makerobo_tempPIN.close()
            break

        makerobo_analogVal = makerobo_tempPIN.value
        makerobo_Vr = float(makerobo_analogVal) * 3.3
        makerobo_Rt = 10000 * makerobo_Vr / (3.3 - makerobo_Vr)
        makerobo_Temp = 1 / (((math.log(makerobo_Rt / 10000)) / 3950) + (1 / (273.15 + 25)))
        temp_c = makerobo_Temp - 273.15
        print(f"Temp C = {temp_c:.2f}")

        # Store values and write them to CSV
        x.append((current_time - start_time) / 60)  # Convert to minutes
        y.append(temp_c)
        csv_writer.writerow([x[-1], y[-1]])  # Write to CSV

        makerobo_tmp = not makerobo_DO.is_pressed
        if makerobo_tmp != makerobo_Status:
            makerobo_print(makerobo_tmp)
            makerobo_Status = makerobo_tmp

        # Update the graph
        plt.cla()
        plt.plot(x, y)
        plt.xlabel('Time (min)')
        plt.ylabel('Temp_C')
        plt.ylim([20, 50])
        plt.pause(0.05)

        sleep(10)


if __name__ == '__main__':
    try:
        makerobo_loop(600)  # GPIO will be released after 10 minutes
    except KeyboardInterrupt:
        pass

plt.show()


