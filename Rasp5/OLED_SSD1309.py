from luma.core.interface.serial import spi
from luma.core.render import canvas
from luma.oled.device import ssd1309
import time

# SPI接続の設定
serial = spi(port=1, device=0, gpio_DC=13, gpio_RST=14, gpio_CS=18)

# SSD1309 OLEDディスプレイの設定
device = ssd1309(serial)

# 画面サイズ
width = device.width
height = device.height

# オブジェクトの初期位置と速度
x, y = 10, 10
vx, vy = 1, 1

# デモプログラムのループ
try:
    while True:
        # 画面をクリア
        with canvas(device) as draw:
            # オブジェクトを描画
            draw.rectangle((x, y, x + 10, y + 10), outline="white", fill="white")

        # オブジェクトの位置を更新
        x += vx
        y += vy

        # 壁に当たったら反射
        if x <= 0 or x >= width - 10:
            vx = -vx
        if y <= 0 or y >= height - 10:
            vy = -vy

        # 少し待つ
        time.sleep(0.01)

except KeyboardInterrupt:
    pass
