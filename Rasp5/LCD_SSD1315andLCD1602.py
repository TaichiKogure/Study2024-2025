import time
from datetime import datetime
from RPLCD.i2c import CharLCD

# LCDの初期化
lcd = CharLCD(i2c_expander='PCF8574', address=0x27)
lcd.clear()

while True:
    # 現在の時刻の取得
    current_time = datetime.now().strftime('%H:%M:%S')
    lcd.write_string('Time: {}'.format(current_time))
    # 1秒待機
    time.sleep(1)
    lcd.clear()


import time
from datetime import datetime
from Adafruit_SSD1306 import SSD1306_64_32

# Create the I2C interface.
i2c = board.I2C()

# 64x32ディスプレイでOLEDの初期化
disp = adafruit_ssd1306.SSD1306_I2C(64, 32, i2c, addr=0x3C)

# Clear display.
disp.fill(0)
disp.show()

font = ImageFont.load_default()

while True:
    # Clear display.
    disp.fill(0)
    disp.show()

    now = datetime.now()
    current_time = now.strftime("%H:%M:%S")

    # Draw some text.
    image = Image.new("1", (disp.width, disp.height))
    draw = ImageDraw.Draw(image)
    draw.text((0, 0), "Time: " + current_time, font=font, fill=255)

    # Display image.
    disp.image(image)
    disp.show()
    # Sleep for a bit.
    time.sleep(1)
