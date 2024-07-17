from gpiozero import Buzzer
from gpiozero import LED
from time import sleep
from signal import pause
import threading

makerobo_buzzer = Buzzer(4)

def exit_script():
    makerobo_destroy()
    exit(0)

def makerobo_setup():
    global bz
    bz = Buzzer(pin=makerobo_buzzer,active_high= False)
    bz.off()

def makerobo_buzzer_on():
    bz.on()

def makerobo_buzzer_off():
    bz.off()

def makerobo_beep(x):
    makerobo_buzzer_on()
    sleep(x)
    makerobo_buzzer_off()
    sleep(x)

def loop():
    while True:
        makerobo_beep(0.5)

def makerobo_destroy():
    bz.close()

exit_timer = threading.Timer(60, exit_script)
exit_timer.start()

if __name__ == '__main__':
    makerobo_setup()
    try:
        loop()
    except KeyboardInterrupt:
        pass
    finally:
        makerobo_destroy()