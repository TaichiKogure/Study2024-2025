import gpiozero
from gpiozero import Button
from gpiozero import LED
from time import sleep
from signal import pause
import threading


makerobo_TiltPin = Button(17)
p_R = LED(18)
p_G = LED(19)


def pressed():
    print('******************************')
    print('*make robo Raspberry Kit Tilt*')
    print('******************************')
    p_G.on()
    p_R.off()


def released():
    p_G.off()
    p_R.on()


makerobo_TiltPin.when_pressed = pressed
makerobo_TiltPin.when_released = released


def exit_script():
    makerobo_destroy()
    exit(0)

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
