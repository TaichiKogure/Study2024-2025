from gpiozero import Button
from gpiozero import LED
from time import sleep
from signal import pause
import threading

makerobo_VibratePin = Button(17)
p_R = LED(18)
p_B = LED(27)


def exit_script():
    makerobo_destroy()
    exit(0)
def pressed():
    print("Button On")
    p_R.on()
    p_B.off()

def released():
    print("Button Off")
    p_R.off()
    p_B.on()

makerobo_VibratePin.when_pressed = pressed
makerobo_VibratePin.when_released = released

def makerobo_loop():
    pause()

def makerobo_destroy():
    p_R.close()
    p_B.close()

exit_timer = threading.Timer(60, exit_script)
exit_timer.start()

if __name__ == '__main__':
    try:
        makerobo_loop()
    except KeyboardInterrupt:
        pass
    finally:
        makerobo_destroy()