from gpiozero import Button
from gpiozero import LED
from time import sleep
from signal import pause

# Button and LED pin assignment
btn = Button(17)
p_R = LED(18)
p_G = LED(19)


# initial state

def pressed():
    print('#######################################')
    print('#makerobo Raspberry Kit Button Pressed#"')
    print('#######################################')
    p_R.on()
    p_G.off()


def released():
    print('Button was released')
    p_R.off()
    p_G.on()


btn.when_pressed = pressed
btn.when_released = released


def makerobo_loop():
    pause()


def makerobo_destroy():
    p_R.close()
    p_G.close()


if __name__ == '__main__':
    try:
        makerobo_loop()
    except KeyboardInterrupt:
        makerobo_destroy()
