from gpiozero import PWMLED
from time import sleep

colors = [0xFF00, 0x00FF, 0x0FF0, 0xF00F]
makerobo_pins = (17, 18)

p_R = PWMLED(pin =makerobo_pins[0],initial_value = 0.2, frequency = 2000)
p_G = PWMLED(pin =makerobo_pins[1],initial_value = 0, frequency = 4500)

def makerobo_pwn_map(x, in_min, in_max, out_min, out_max):
    return (x-in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def makerobo_set_Color(col):
    R_val = col >> 8
    G_val = col & 0x00FF
    R_val = makerobo_pwn_map(R_val, 0,255,0,100)
    G_val = makerobo_pwn_map(G_val, 0,255,0,100)

    p_R.value = (R_val)/100.0
    p_G.value = (G_val)/100.0

def makerobo_roop():
    while True:
        for col in colors:
            makerobo_set_Color(col)
            sleep(0.05)

def makerobo_destroy():
    p_G.close()
    p_R.close()

if __name__ == "__main__":
    try:
        makerobo_roop()
    except KeyboardInterrupt:
        makerobo_destroy()