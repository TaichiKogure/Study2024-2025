import threading
from gpiozero import Button, LED
from time import sleep
from signal import pause


class MakeroboButtonLED:
    def __init__(self, button_pin, led_pin_red, led_pin_green, timeout):
        self.btn = Button(button_pin)
        self.led_red = LED(led_pin_red)
        self.led_green = LED(led_pin_green)
        self.btn.when_pressed = self._pressed
        self.btn.when_released = self._released
        self.timeout = timeout

    def _print_message(self, message):
        print('#######################################')
        print(message)
        print('#######################################')

    def _pressed(self):
        self._print_message('# Makerobo Raspberry Kit Button Pressed #')
        self.led_red.on()
        self.led_green.off()

    def _released(self):
        print('Button was released')
        self.led_red.off()
        self.led_green.on()

    def loop(self):
        threading.Timer(self.timeout * 10, self.destroy).start()
        pause()

    def destroy(self):
        self.led_red.close()
        self.led_green.close()


if __name__ == '__main__':
    makerobo_obj = MakeroboButtonLED(17, 18, 19, 1)
    try:
        makerobo_obj.loop()
    except KeyboardInterrupt:
        makerobo_obj.destroy()
