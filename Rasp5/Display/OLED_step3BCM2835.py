# Note: This method involves using C extensions and might be less
# straightforward than using RPi.GPIO.
# Use luma.oled compatible package that internally uses bcm2835 library

from luma.core.interface.serial import spi
from luma.core.render import canvas
from luma.oled.device import ssd1309
from PIL import ImageFont

# SPIインターフェースの設定
serial = spi(device=0, port=0, cs_high=False, transfer_size=4096)

# SSD1309 ディスプレイの初期化(拡張ライブラリを使ったもの)
device = ssd1309(serial, gpio_DC=25, gpio_RST=27)

# ディスプレイにテキストを表示
font = ImageFont.load_default()
with canvas(device) as draw:
    draw.text((0, 0), "Hello, World!", font=font, fill="white")


# ディスプレイにグラフィックを表示 (オプション)
def display_graphics():
    with canvas(device) as draw:
        draw.rectangle(device.bounding_box, outline="white", fill="black")
        draw.text((30, 40), "SSD1309 Test", fill="white")


if __name__ == "__main__":
    display_graphics()
