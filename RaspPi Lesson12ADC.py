from gpiozero import PWMLED, MCP3008
from gpiozero.tools import absoluted, scaled
from signal import pause
import threading



led = PWMLED(17)
pot = MCP3008(channel=0)

if __name__ == '__main__':
    try:
        led.source = absoluted(pot)
        pause()
    except KeyboardInterrupt:
        led.close()
        raise