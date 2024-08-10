from gpiozero import Button, MCP3008
from gpiozero.tools import absoluted, scaled
from signal import pause
from time import sleep

makerobo_DO = Button(17)
pot = MCP3008(channel=0)


def pressed():
    print('Raining!')


def released():
    print('Not Raining')


makerobo_DO.when_pressed = pressed
makerobo_DO.when_released = released


def MAP(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def makerobo_loop():
    pot_vlue = pot.value
    pot_vlue = MAP(pot_vlue, 0, 1, 0, 100)
    print("{:.2f}".format(pot_vlue))
    sleep(0.2)


if __name__ == '__main__':
    try:
        makerobo_loop()
    except KeyboardInterrupt:
        makerobo_DO.close()
