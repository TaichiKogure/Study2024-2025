# Lesson17+25+27

from gpiozero import Button, MCP3008
import matplotlib.pyplot as plt
from time import sleep, time
import math
import csv
import threading

# for Lesson 27
import board
import adafruit_dht
makerobo_dhtDevice = adafruit_dht.DHT11(board.D16)


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
y3 = []
H1 = []

# csvファイル名設定
csvfile = "Temp_Data_2sensor-3.csv"
csv_writer = csv.writer(open(csvfile, "w"))
csv_writer.writerow(["time", "temp_C_sensor1", "temp_C_sensor2","temp_C_sensor3","humidity"])


#def makerobo_print(x):
    # if x == 1:
    #     print('Better')
    # if x == 0:
    #     print('Too Hot !!')

def makerobo_loop(runtime, time):
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

        makerobo_tmp = not makerobo_DO.is_pressed
        if makerobo_tmp != makerobo_Status:
            makerobo_print(makerobo_tmp)
            makerobo_Status = makerobo_tmp


        if makerobo_read() != None:
            print("Current tempereture on Sensor2 : %0.3f C" % makerobo_read())

        #Lesson27 module
        try:
            TEMP_C_with_Humidity = makerobo_dhtDevice.temperature
            humidity = makerobo_dhtDevice.humidity
            print("Temperature on Sensor3 : {:.f} C Humidity:{}% ".format(TEMP_C_with_Humidity, humidity))
        except RuntimeError as error:
            print(error.args[0])
            time.sleep(2)
            continue

        except Exception as error:
            makerobo_dhtDevice.exit()
            raise error
        time.sleep(2)

        # Update the graph
        plt.cla()  # Clear axis is not necessary as both plots are on same graph 
        plt.plot(x, y1, label='Temp_C_S1')
        plt.plot(x, y2, label='Temp_C_S2')
        plt.plot(x, y3, label='Temp_C_S3')
        plt.xlabel('Time')
        plt.ylabel('Temperature (Celsius)')
        plt.ylim([20, 35])  # Assumes similar range for both sensors
        plt.legend()  # Displays the legend on the plot
        plt.pause(0.1)

        plt.cla()  # Clear axis is not necessary as both plots are on same graph
        plt.plot(x, H1, label='Temp_C_S1')

        plt.xlabel('Time')
        plt.ylabel('Humidity[%]')
        plt.ylim([20, 100])  # Assumes similar range for both sensors
        plt.legend()  # Displays the legend on the plot
        plt.pause(0.1)

        sleep(10)
        
        x.append((current_time - start_time)/60)
        y1.append(temp_c)
        y2.append(makerobo_read())
        y3.append(TEMP_C_with_Humidity)
        H1.append(humidity)

        csv_writer.writerow([x[-1], y1[-1], y2[-1]], y3[-1], H1[-1])  # csvに書き出し
        
def destroy():
    pass

if __name__ == '__main__':
    try:
        makerobo_setup()
        makerobo_loop(1440)  # XX分後にGPIO解放されます
    except KeyboardInterrupt:
        pass

plt.show()