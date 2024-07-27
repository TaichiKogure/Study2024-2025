from gpiozero import Button, MCP3008
from gpiozero.tools import absoluted, scaled
from signal import pause
from time import sleep
import math

import matplotlib.pyplot as plt
from collections import deque

# 最大値と最小値を設定
y_range = [15.0, 40.0]

x = deque(maxlen=60)  # deque maxlenを指定することで、自動的に要素数が制限される（古いデータが消える）
y = deque(maxlen=60)  # deque maxlenを指定することで、自動的に要素数が制限される（古いデータが消える）

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
    # matplotlibの初期設定
    plt.ion()  # interactive modeをオンにする
    fig, ax = plt.subplots()

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

        # データをdequeに追加する
        x.append(temp_c)
        y.append(temp_f)

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


