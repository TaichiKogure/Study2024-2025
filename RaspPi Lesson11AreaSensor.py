from gpiozero import Button
from gpiozero import LED
from time import sleep
from signal import pause
import threading


def exit_script():
    makerobo_destroy()
    exit(0)


makerobo_PIPIn = Button(17)
p_R = LED(18)
p_G = LED(27)


def pressed():
    print('Makerobo Light was blocked ')
    p_R.off()
    p_G.on()


def released():
    p_R.on()
    p_G.off()


makerobo_PIPIn.when_pressed = pressed
makerobo_PIPIn.when_released = released


def makerobo_loop():
    pause()


def makerobo_destroy():
    p_R.close()
    p_G.close()


exit_timer = threading.Timer(60, exit_script)
exit_timer.start()

if __name__ == '__main__':
    try:
        makerobo_loop()
    except KeyboardInterrupt:
        pass
    finally:
        makerobo_destroy()
