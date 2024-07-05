from gpiozero import LED
from time import sleep

makerobo_laserPIN = LED(16)
def makerobo_setup():
    makerobo_laserPIN.off()

def makerobo_loop():
    while True:
        makerobo_laserPIN.on()
        sleep(0.5)

        makerobo_laser PIN.off()
        sleep(0.5)

def makerobo_destroy():
    makerobo_laserPIN.close()

if __name__ == '__main__':
    makerobo_setup()
    try:
        makerobo_loop()
    except KeyboardInterrupt:
        makerobo_destroy()
