from gpiozero import Button, MCP3008
from gpiozero.tools import absoluted, scaled
from signal import pause
from time import sleep
import math


makerobo_DO = Button(17)
makerobo_tempPIN = MCP3008(channel=0)

def makerobo_Print(x):
    if x == 1:
        print('Better')
    if x == 0:
        print('Too Hot !!')

def MAP(x, in_min, in_max, out_min, out_max):
    return (x- in_min)*(out_max - out_min)/(in_max - in_min) + out_min

def makerobo_loop():

    makerobo_Status = 1
    makerobo_tmp = 1
    while True:
        makerobo_analogVal = makerobo_tempPIN.value
        makerobo_Vr = float(makerobo_analogVal) * 3.3
        makerobo_Rt = 10000 * makerobo_Vr / (3.3 - makerobo_Vr)
        makerobo_Temp = 1/(((math.log(makerobo_Rt / 10000)) /3950) + (1 / (273.15+25)))
        temp_c = makerobo_Temp -273.15
        temp_f = temp_c * 9.0 /5.0 + 32
        print("Temp C = {:.2f} ¥t Temp F = {:.2f}".format(temp_c, temp_f))



        makerobo_tmp = not makerobo_DO.is_pressed
        if makerobo_tmp != makerobo_Status:
            makerobo_Print(makerobo_tmp)
            makerobo_Status = makerobo_tmp

        # グラフの更新
        ax.clear()  # 描画クリア
        ax.plot(x, y)  # データをプロット
        ax.set_ylim(y_range)  # y軸の範囲を設定
        plt.pause(0.02)  # 描画が更新されるように一瞬だけプログラムを停止

        sleep(10)




if __name__ == '__main__':
    try:
        makerobo_loop()
    except KeyboardInterrupt:
        pass


#%%
from ipywidgets import IntSlider, HBox, VBox
import threading
from IPython.display import display

makerobo_DO = Button(17)
makerobo_tempPIN = MCP3008(channel=0)
data_slider = IntSlider(description='Temperature', min=-10, max=50)


def update_data_slider():
    makerobo_Status = 1
    makerobo_tmp = 1
    while True:
        makerobo_analogVal = makerobo_tempPIN.value
        makerobo_Vr = float(makerobo_analogVal) * 3.3
        makerobo_Rt = 10000 * makerobo_Vr / (3.3 - makerobo_Vr)
        makerobo_Temp = 1 / (((math.log(makerobo_Rt / 10000)) / 3950) + (1 / (273.15 + 25)))
        temp_c = makerobo_Temp - 273.15
        data_slider.value = int(temp_c)  # Update the slider
        sleep(1)


# Create and start thread for data update
thread = threading.Thread(target=update_data_slider)
thread.start()

# Display widgets
display(VBox([data_slider]))

#%%
from gpiozero import Button, MCP3008
import matplotlib.pyplot as plt
from time import sleep, time
import math
import csv
import threading

# GPIO設定
makerobo_DO = Button(17)
makerobo_tempPIN = MCP3008(channel=0)

# グラフのx軸、y軸を保存するリスト設定
x = []
y = []

# csvファイル名設定
csvfile = "temp_data.csv"
csv_writer = csv.writer(open(csvfile, "w"))
csv_writer.writerow(["time(min)", "temp_C"])


def makerobo_print(x):
    if x == 1:
        print('Better')
    if x == 0:
        print('Too Hot !!')


def make_graph():
    while True:
        plt.cla()
        plt.plot(x, y)
        plt.xlabel('Time (min)')
        plt.ylabel('Temp_C')
        plt.ylim([0, 50])
        plt.pause(0.05)


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

        x.append(current_time - start_time)
        y.append(temp_c)
        csv_writer.writerow([x[-1], y[-1]])  # csvに書き出し

        makerobo_tmp = not makerobo_DO.is_pressed
        if makerobo_tmp != makerobo_Status:
            makerobo_print(makerobo_tmp)
            makerobo_Status = makerobo_tmp

        sleep(10)


# グラフ作成スレッド作成
graph_thread = threading.Thread(target=make_graph)
graph_thread.start()

if __name__ == '__main__':
    try:
        makerobo_loop(10)  # 10分後にGPIO解放されます
    except KeyboardInterrupt:
        pass

plt.show()

##%
# Repair 2
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
        plt.ylim([0, 50])
        plt.pause(0.05)

        sleep(10)


if __name__ == '__main__':
    try:
        makerobo_loop(10)  # GPIO will be released after 10 minutes
    except KeyboardInterrupt:
        pass

plt.show()


