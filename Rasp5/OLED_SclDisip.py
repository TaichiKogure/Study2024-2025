import time
import board
from adafruit_ssd1306 import SSD1306_I2C
from datetime import datetime
from PIL import Image, ImageDraw, ImageFont

# Create the I2C interface.
i2c = board.I2C()

# 64x32ディスプレイでOLEDの初期化
disp = SSD1306_I2C(64, 32, i2c)

# Clear display.
disp.fill(0)
disp.show()

font = ImageFont.load_default()

scroll_text = " Date: {0} Time: {1} ".format(datetime.now().strftime('%Y-%m-%d'), datetime.now().strftime('%H:%M:%S'))
start_pos = 64
end_pos = 0

while True:
    # Clear display.
    disp.fill(0)
    disp.show()

    for i in range(start_pos, end_pos, -1):
        image = Image.new("1", (disp.width, disp.height))
        draw = ImageDraw.Draw(image)
        draw.text((i, 0), scroll_text, font=font, fill=255)

        disp.image(image)
        disp.show()
        time.sleep(0.1)

    start_pos = 64
