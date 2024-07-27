from gpiozero import Button, MCP3008
from gpiozero.tools import absoluted, scaled
from signal import pause
from time import sleep

pot_x = MCP3008(channel=0)
pot_y = MCP3008(channel=1)
pot_z = MCP3008(channel=2)

def MAP(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def makerobo_direction():
    state = ['home', 'up', 'down', 'left', 'right', 'pressed']
    i = 0
    x_value = round(pot_x.value * 1024)
    y_value = round(pot_y.value * 1024)
    z_value = round(pot_z.value * 1024)
    x_value = round(MAP(x_value, 0, 1023, 0, 255))
    y__value = round(MAP(y_value, 0, 1023, 0, 255))
    z_value = round(MAP(z_value, 0, 1023, 0, 255))
    '''
    print('x_value:{},y_value:{},z_value:{}'.format(x_value,y_value,z_value))
    '''
    if x_value <= 30:
        i = 1
    if x_value >= 225:
        i = 2
    if y_value >= 225:
        i = 4
    if y_value <= 30:
        i = 3
    if z_value <= 2 and y_value >= 128:
        i = 5
    if 15 > x_value - 125 > -15 and 15 > y_value - 125 > -15 and z_value >= 250:
        i = 0

    return state[i]


def makerobo_loop():
    makerobo_status = ''
    while True:
        makerobo_tmp = makerobo_direction()
        sleep(0.2)
        if makerobo_tmp != None and makerobo_tmp != makerobo_status:
            print(makerobo_tmp)
            makerobo_status = makerobo_tmp


def destroy():
    pass


if __name__ == '__main__':
    try:
        makerobo_loop()
    except KeyboardInterrupt:
        destroy()
