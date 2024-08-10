#%%
# from gpiozero import PWMLED
import time
from time import sleep
import random

import matplotlib.pyplot as plt
from collections import namedtuple

from gpiozero import PWMLED

# Time, Color and Brightness??????????
TCB_Record = namedtuple('TCB_Record', 'time color brightness')

# ???????????????
record_list = []

colors = [0xFF00, 0x00FF, 0x0FF0, 0xF00F]
makerobo_pins = (17, 18)

p_R = PWMLED(pin =makerobo_pins[0],initial_value = 0.2, frequency = 2000)
p_G = PWMLED(pin =makerobo_pins[1],initial_value = 0, frequency = 4500)

def makerobo_pwn_map(x, in_min, in_max, out_min, out_max):
    return (x-in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def makerobo_set_Color(col,brightness):
    R_val = (col >> 8) * brightness
    G_val = (col & 0x00FF) * brightness
    R_val = makerobo_pwn_map(R_val, 0, 255, 0, 100)
    G_val = makerobo_pwn_map(G_val, 0, 255, 0, 100)

    # ????R_val?G_val???????????
    R_val *= random.uniform(0, 1)
    G_val *= random.uniform(0, 1)

    p_R.value = (R_val) / 100.0
    p_G.value = (G_val) / 100.0


def makerobo_roop():
    start_time = time.time()
    while True:
        for col in colors:
            brightness = random.uniform(0.01, 1)  #
            makerobo_set_Color(col, brightness)  #
            blink_time = random.uniform(0.1, 0.8)
            sleep(blink_time)

            #
            elapsed_time = time.time() - start_time  #
            record_list.append(TCB_Record(elapsed_time, col, brightness))

            makerobo_set_Color(0, 0)  # ??
            off_time = random.uniform(0.1, 0.5)
            sleep(off_time)

            #
            elapsed_time = time.time() - start_time  # ???????
            record_list.append(TCB_Record(elapsed_time, 0, 0))
            #
            times = [record.time for record in record_list]
            brightnesses = [record.brightness for record in record_list]

            plt.plot(times, brightnesses)

            plt.xlabel('Time (s)')
            plt.ylabel('Brightness')
            plt.title('Brightness over Time')
            plt.show()
def makerobo_destroy():
    p_G.close()
    p_R.close()




if __name__ == "__main__":
    try:
        makerobo_roop()
    except KeyboardInterrupt:
        makerobo_destroy()


#%%


# from gpiozero import PWMLED
import time
from time import sleep
import random

import matplotlib.pyplot as plt
from collections import namedtuple

# Time, Color and Brightness??????????
TCB_Record = namedtuple('TCB_Record', 'time color brightness R_val G_val')

# ???????????????
record_list = []

colors = [0xFF00, 0x00FF, 0x0FF0, 0xF00F]
makerobo_pins = (17, 18)

p_R = PWMLED(pin =makerobo_pins[0],initial_value = 0.2, frequency = 2000)
p_G = PWMLED(pin =makerobo_pins[1],initial_value = 0, frequency = 4500)

def makerobo_pwn_map(x, in_min, in_max, out_min, out_max):
    return (x-in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def makerobo_set_Color(col, brightness):
    R_val = (col >> 8) * brightness
    G_val = (col & 0x00FF) * brightness
    R_val = makerobo_pwn_map(R_val, 0, 255, 0, 100)
    G_val = makerobo_pwn_map(G_val, 0, 255, 0, 100)

    # 让R_val和G_val的值在范围内随机浮动
    R_val *= random.uniform(0, 1)
    G_val *= random.uniform(0, 1)

    p_R.value = (R_val) / 100.0
    p_G.value = (G_val) / 100.0

    return R_val, G_val


def makerobo_roop():
    start_time = time.time()
    while True:
        for col in colors:
            brightness = random.uniform(0.01, 1)
            R_val, G_val = makerobo_set_Color(col, brightness)
            blink_time = random.uniform(0.1, 0.8)
            sleep(blink_time)
            elapsed_time = time.time() - start_time  #

            record_list.append(TCB_Record(elapsed_time, col, brightness, R_val, G_val))

            R_val,G_val = makerobo_set_Color(0, 0)  # ??
            off_time = random.uniform(0.1, 0.5)
            sleep(off_time)
            elapsed_time = time.time() - start_time  # ???????
            record_list.append(TCB_Record(elapsed_time, 0, 0, R_val,G_val))
            #
            times = [record.time for record in record_list]
            brightnesses = [record.brightness for record in record_list]
            R_vals = [record.R_val for record in record_list]
            G_vals = [record.G_val for record in record_list]
            # Create the figure and the subplots
            fig, axs = plt.subplots(nrows=1, ncols=2, figsize=(12, 4))

            # Plot the brightness over time on the first subplot
            axs[0].scatter(times, brightnesses)
            axs[0].set_xlabel('Time (s)')
            axs[0].set_ylabel('Brightness')
            axs[0].set_title('Brightness over Time')

            # Plot the R_val and G_val over time on the second subplot
            axs[1].scatter(times, R_vals, label='R')
            axs[1].scatter(times, G_vals, label='G')
            axs[1].set_xlabel('Time (s)')
            axs[1].set_ylabel('Value')
            axs[1].set_title('R_val and G_val over Time')
            axs[1].legend()

            # Display the plots
            plt.tight_layout()
            plt.show()


def makerobo_destroy():
    p_G.close()
    p_R.close()




if __name__ == "__main__":
    try:
        makerobo_roop()
    except KeyboardInterrupt:
        makerobo_destroy()