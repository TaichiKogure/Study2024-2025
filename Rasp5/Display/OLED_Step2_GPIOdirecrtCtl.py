import RPi.GPIO as GPIO
from luma.core.interface.serial import spi
from luma.core.render import canvas
from luma.oled.device import ssd1309
from PIL import ImageFont

# GPIOピンの設定
try:
    GPIO.setmode(GPIO.BCM)
    DC_PIN = 25
    RST_PIN = 27

    # Define the GPIO pins
    GPIO.setup(DC_PIN, GPIO.OUT)
    GPIO.setup(RST_PIN, GPIO.OUT)
    print("GPIO setup success")
except RuntimeError as e:
    print(f"GPIO setup failed: {e}")
    exit(1)

# SPIインターフェースの設定（GPIOピンの指定）
serial = spi(device=0, port=0, cs_high=False, transfer_size=4096)

# SSD1309 ディスプレイの初期化
device = ssd1309(serial, gpio_DC=DC_PIN, gpio_RST=RST_PIN)

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
    GPIO.cleanup()
