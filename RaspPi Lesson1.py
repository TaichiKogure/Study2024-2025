from gpiozero import PWMLED
from time import sleep
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

colors = [0xFF00, 0x00FF, 0x0FF0, 0xF00F]
makerobo_pins = (17, 18)

p_R = PWMLED(pin =makerobo_pins[0],initial_value = 0.2, frequency = 2000)
p_G = PWMLED(pin =makerobo_pins[1],initial_value = 0, frequency = 4500)

def makerobo_pwn_map(x, in_min, in_max, out_min, out_max):
    return (x-in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def makerobo_set_Color(col):
    R_val = col >> 8
    G_val = col & 0x00FF
    R_val = makerobo_pwn_map(R_val, 0, 255, 0, 100)
    G_val = makerobo_pwn_map(G_val, 0, 255, 0, 100)

    # 新たに、R_valとG_valをランダムに調整します
    R_val *= random.uniform(0, 1)
    G_val *= random.uniform(0, 1)

    p_R.value = (R_val) / 100.0
    p_G.value = (G_val) / 100.0


def makerobo_roop():
    start_time = time.time()
    while True:
        for col in colors:
            brightness = random.uniform(0, 1)
            makerobo_set_Color(col, brightness)
            blink_time = random.uniform(0.1, 1)
            sleep(blink_time)

            elapsed_time = time.time() - start_time
            record_list.append(TCB_Record(elapsed_time, col, brightness))

            makerobo_set_Color(0, 0)
            off_time = random.uniform(0.1, 1)
            sleep(off_time)

            elapsed_time = time.time() - start_time
            record_list.append(TCB_Record(elapsed_time, 0, 0))



def make_plot():
    # Initialize figure
    fig, ax = plt.subplots()

    # Update function for the animation
    def update(num):
        times = [record.time for record in record_list]
        brightnesses = [record.brightness for record in record_list]
        ax.clear()
        ax.plot(times, brightnesses)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Brightness')
        ax.set_title('Brightness over Time')

    ani = animation.FuncAnimation(fig, update, frames=100, repeat=True)
    plt.show()


if __name__ == "__main__":
    try:
        make_plot()  # call plot function
        makerobo_roop()
    except KeyboardInterrupt:
        makerobo_destroy()
def makerobo_destroy():
    p_G.close()
    p_R.close()

