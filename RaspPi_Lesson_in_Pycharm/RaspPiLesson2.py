from gpiozero import RGBLED
from colorzero import Color
from time import sleep

colors = ['red', 'green', 'blue','yellow','magenta','cyan']

led = RGBLED(23,24,27)

def makerobo_loop():
    while True:
        for col in colors:
            led.color = Color(col)
            sleep(0.5)

def makerobo_destroy():
    led.close()

if __name__ == '__main__':
    try:
        makerobo_loop()
    except KeyboardInterrupt:
        makerobo_destroy()
