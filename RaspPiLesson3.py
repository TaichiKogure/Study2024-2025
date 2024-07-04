from gpiozero import LED
from time import sleep

makerobo_RalayPin = LED(17)

def makerobo_setup():
    makerobo_RalayPin.off()


def makerobo_loop():
    while True:
        makerobo_RalayPin.on()
        sleep(0.5)

        makerobo_RalayPin.off()
        sleep(0.5)

def makerobo_destroy():
    makerobo_RalayPin.close()

if __name__ == '__main__':
    makerobo_setup()
    try:
        makerobo_loop()
    except KeyboardInterrupt:
        makerobo_destroy()
