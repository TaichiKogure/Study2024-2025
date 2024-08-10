from gpiozero import Motor
from time import sleep

Motor = Motor(forward=16, backward=17, enable=18)

fs_directions = {'clockwiseOpen': Motor.FORWARD,'anticlockwiseOpen': Motor.BACKWARD,'STOP':Motor.STOP}

def makerobo_loop():
    while True:
        for action in ['clockwiseOpen','STOP','anticlockwiseOpen','STOP']:
            fs = fs_directions[action]()
            print(f"{action}")
            sleep(5)

def destroy():
    Motor.stop()

if __name__ == '__main__':
    try:
        makerobo_loop()
    except KeyboardInterrupt:
        destroy()


