from gpiozero import RGBLED
from time import sleep

import threading
from gpiozero import RGBLED
from time import sleep

led = RGBLED(red=2, green=3, blue=4)


def color_cycle(cycle_time):
    for i in range(100):
        # Red to yellow (increasing green)
        led.red = 1.0
        led.green = i / 100.0
        sleep(cycle_time)

    for i in range(100):
        # Yellow to green (reducing red)
        led.red = 1.0 - i / 100.0
        led.green = 1.0
        sleep(cycle_time)

    for i in range(100):
        # Green to cyan (increasing blue)
        led.green = 1.0
        led.blue = i / 100.0
        sleep(cycle_time)

    for i in range(100):
        # Cyan to blue (reducing green)
        led.green = 1.0 - i / 100.0
        led.blue = 1.0
        sleep(cycle_time)

    for i in range(100):
        # Blue to red (reducing blue and increasing red)
        led.red = i / 100.0
        led.blue = 1.0 - i / 100.0
        sleep(cycle_time)


def stop_color_cycle():
    global is_running
    is_running = False
    print("Ending the color cycle.")
    led.close()


is_running = True

if __name__ == '__main__':
    # stop after 60 seconds
    stop_timer = threading.Timer(60.0, stop_color_cycle)
    stop_timer.start()

    try:
        while is_running:
            color_cycle(0.05)
    except KeyboardInterrupt:
        print("Interrupted by user.")
        led.close()
