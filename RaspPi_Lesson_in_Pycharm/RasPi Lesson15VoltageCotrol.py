from gpiozero import PWMLED, MCP3008
from gpiozero.tools import absoluted, scaled
from signal import pause
from time import sleep

pot = MCP3008(channel=0)
r_LED = PWMLED(17)

def MAP(x,in_min, in_max, out_min, out_max):
    return (x-in_min)/(in_max-in_min)*out_max-out_min

def makerobo_loop():
    makerobo_status = 1
    while True:
        pot_value = round(pot.value * 1000)
        print('Potentiometer Value:', pot_value)
        r_LED.source = absoluted(pot)
        sleep(0.5)

def destroy():
    r_LED.close()

if __name__ == '__main__':
    try:
        makerobo_loop()
    except KeyboardInterrupt:
        destroy()
