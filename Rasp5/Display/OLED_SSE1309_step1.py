from luma.core.interface.serial import spi
from luma.core.render import canvas
from luma.oled.device import ssd1309
from PIL import ImageFont

# SPIインターフェースの設定（GPIOピンの指定）
serial = spi(device=0, port=0, cs_high=False, transfer_size=4096,
             gpio_DC=25, gpio_RST=27)

# SSD1309 ディスプレイの初期化
device = ssd1309(serial)

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
