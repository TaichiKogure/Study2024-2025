from machine import Pin, SPI
import framebuf
import time

# ピンの設定
DC_PIN = 13
RST_PIN = 12
CS_PIN = 5

# SPIの初期化
spi = SPI(0, baudrate=500000, polarity=0, phase=0, sck=Pin(2), mosi=Pin(3))


# 初期化関数
def oled_init(dc_pin, rst_pin, cs_pin):
    dc = Pin(dc_pin, Pin.OUT)
    rst = Pin(rst_pin, Pin.OUT)
    cs = Pin(cs_pin, Pin.OUT)

    # SSD1309 ディスプレイの初期化コマンド
    init_cmds = [
        0xAE,  # Display off
        0xD5, 0x80,  # Set display clock division
        0xA8, 0x3F,  # Set multiplex ratio (1 to 64)
        0xD3, 0x00,  # Set display offset
        0x40,  # Set start line address
        0x8D, 0x14,  # Charge Pump
        0x20, 0x00,  # Memory mode
        0xA1,  # Segment re-map
        0xC8,  # COM output scan direction
        0xDA, 0x12,  # Set com pins hardware configuration
        0x81, 0xCF,  # Set contrast control register
        0xD9, 0xF1,  # Set pre-charge period
        0xDB, 0x40,  # Set vcomh
        0xA4,  # Output follows RAM content
        0xA6,  # Normal display
        0xAF  # Display on
    ]

    # リセット
    rst.value(0)
    time.sleep(0.01)
    rst.value(1)

    # コマンド送信
    for cmd in init_cmds:
        cs.value(0)
        dc.value(0)  # Command mode
        spi.write(bytearray([cmd]))
        cs.value(1)

    return dc, cs


# OLEDの初期化
dc, cs = oled_init(DC_PIN, RST_PIN, CS_PIN)

# フレームバッファ
width = 128
height = 64
buffer = bytearray(width * height // 8)
fb = framebuf.FrameBuffer(buffer, width, height, framebuf.MONO_VLSB)  # 試行: MONO_VLSB

# テストメッセージを表示
fb.fill(0)
fb.text('OK,COOL!! 0123456789abcdefghij!', 0, 0, 1)
fb.text('Gooddisplayabcde', 0, 10, 1)
fb.text('BadHappy this home def', 0, 20, 1)
fb.text('Maddisplayabcdef', 0, 30, 1)
fb.text('Mad-Display-x-y-z-a-b', 0, 40, 1)
fb.text('Display-a-b-c-defg', 0, 50, 1)
fb.text('OK', 0, 60, 1)


# ディスプレイに送信
def update_display():
    for page in range(0, 8):
        cs.value(0)
        dc.value(0)  # Command mode
        spi.write(bytearray([0xB0 + page]))  # Set page address
        spi.write(bytearray([0x00]))  # Set lower column address
        spi.write(bytearray([0x10]))  # Set higher column address
        cs.value(1)

        cs.value(0)
        dc.value(1)  # Data mode
        spi.write(buffer[page * 128:(page + 1) * 128])
        cs.value(1)


update_display()

