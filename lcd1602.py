#LCD 1602 I2C Driver
import utime as time
from umachine import I2C

LCD_CLEARDISPLAY   = 0x01
LCD_RETURNHOME     = 0x02
LCD_ENTRYMODESET   = 0x04
LCD_DISPLAYCONTROL = 0x08

LCD_CURSORSHIFT    = 0x10
LCD_FUNCTIONSET    = 0x20
LCD_SETCGRAMADDR   = 0x40
LCD_SETDDRAMADDR   = 0x80

LCD_ENTRYRIGHT          = 0x00
LCD_ENTRYLEFT           = 0x02
LCD_ENTRYSHIFTINCREMENT = 0x01
LCD_ENTRYSHIFTDECREMENT = 0x00

LCD_DISPLAYON  = 0x04
LCD_DISPLAYOFF = 0x00
LCD_CURSORON   = 0x02
LCD_CURSOROFF  = 0x00
LCD_BLINKON    = 0x01
LCD_BLINKOFF   = 0x00

LCD_DISPLAYMOVE = 0x08
LCD_CURSORMOVE  = 0x00
LCD_MOVERIGHT   = 0x04
LCD_MOVELEFT    = 0x00

LCD_8BITMODE = 0x10
LCD_4BITMODE = 0x00
LCD_2LINE    = 0x08
LCD_1LINE    = 0x00
LCD_5x10DOTS = 0x04
LCD_5x8DOTS  = 0x00

LCD_BACKLIGHT   = 0x08
LCD_NOBACKLIGHT = 0x00

En = 0x04
Rw = 0x02
Rs = 0x01

DISPLAY_DIR_LEFT  = 0
DISPLAY_DIR_RIGHT = 1

TEXT_FLOW_LEFT_TO_RIGHT = 0
TEXT_FLOW_RIGHT_TO_LEFT = 1

class LCD1602Drv:
    def __init__(self, i2c, addr=0x27, cols=16, rows=1, backlight=True):
        self.i2c_addr = addr
        self._displayfunction = (LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS)
        if backlight is True:
            self._backlightVal = LCD_BACKLIGHT
        else:
            self._backlightVal = LCD_NOBACKLIGHT
        self.i2c = i2c
        if rows > 1:
            self._displayfunction |= LCD_2LINE
        self._numlines = rows
        time.sleep_ms(50)

        self._expanderWrite(self._backlightVal)
        time.sleep_ms(1000)

        self._write4Bits(0x03 << 4)
        time.sleep_us(4500)
        self._write4Bits(0x03 << 4)
        time.sleep_us(4500)
        self._write4Bits(0x03 << 4)
        time.sleep_us(150)
        self._write4Bits(0x02 << 4)

        self.sendCommand(LCD_FUNCTIONSET | self._displayfunction)

        self._displayControl = (LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF)
        self.enableDisplay(on=True)
        self.clearScreen()

        self._displayMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT
        self.sendCommand(LCD_ENTRYMODESET | self._displayMode)

        self.returnHome()

    def _expanderWrite(self, _data):
        val = _data | self._backlightVal
        val = val.to_bytes(1, 'little')
        self.i2c.writeto(self.i2c_addr, val)

    def _pulseEnable(self, _data):
        self._expanderWrite(_data | En)
        time.sleep_ms(1)
        self._expanderWrite(_data & ~En)
        time.sleep_ms(50)

    def _write4Bits(self, value):
        self._expanderWrite(value)
        self._pulseEnable(value)

    def sendCommand(self, cmd):
        _mode = 0
        high_nib = cmd & 0xF0
        low_nib = (cmd << 4) & 0xF0
        self._write4Bits(high_nib | _mode)
        self._write4Bits(low_nib | _mode)

    def _pulseEnableAndRs(self, _data):
        self._expanderWrite(_data | En | Rs)
        time.sleep_ms(1)
        self._expanderWrite(_data & ~En | Rs)
        time.sleep_ms(50)

    def sendData(self, data):
        _mode = 0
        high_nib = data & 0xF0
        low_nib = (data << 4) & 0xF0
        self._expanderWrite(high_nib | _mode)
        self._pulseEnableAndRs(high_nib | _mode)
        self._expanderWrite(low_nib | _mode)
        self._pulseEnableAndRs(low_nib | _mode)

    def display(self, string, col=0, row=0):
        if len(string) > 16:
            raise ValueError("String length is too long")
        try:
            self.setCursorPosition(col=col, row=row)
        except ValueError:
            raise ValueError("Move cursor failed")
        for i in string:
            self.sendData(ord(i))

    def enableBackLight(self, on=True):
        if on is True:
            self._backlightVal = LCD_BACKLIGHT
        else:
            self._backlightVal = LCD_NOBACKLIGHT
        self._expanderWrite(0)

    def clearScreen(self):
        self.sendCommand(LCD_CLEARDISPLAY)
        # this command need to wait 1.52ms, choose 2 ms
        time.sleep_ms(2)

    def returnHome(self):
        self.sendCommand(LCD_RETURNHOME)
        # this command need to wait 1.52ms, choose 2 ms
        time.sleep_ms(2)

    def setCursorPosition(self, col=0, row=0):
        offset = [0x00, 0x40, 0x14, 0x54]
        if row > (self._numlines-1):
            raise ValueError("Invlid number of lines")
        self.sendCommand(LCD_SETDDRAMADDR | (col + offset[row]))
        # this command need to wait 37us
        time.sleep_us(37)

    def enableDisplay(self, on=True):
        if on is True:
            self._displayControl |= LCD_DISPLAYON
        else:
            self._displayControl &= ~LCD_DISPLAYON
        self.sendCommand(LCD_DISPLAYCONTROL | self._displayControl)
        time.sleep_us(37)

    def enableCursor(self, on=True):
        if on is True:
            self._displayControl |= LCD_CURSORON
        else:
            self._displayControl &= ~LCD_CURSORON
        self.sendCommand(LCD_DISPLAYCONTROL | self._displayControl)
        time.sleep_us(37)

    def enableBlink(self, on=True):
        if on is True:
            self._displayControl |= LCD_BLINKON
        else:
            self._displayControl &= ~LCD_BLINKON
        self.sendCommand(LCD_DISPLAYCONTROL | self._displayControl)
        time.sleep_us(37)

    def leftShift(self):
        self.sendCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT)

    def rightShift(self):
        self.sendCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT)

    def setTextFlowDir(self, mode=TEXT_FLOW_LEFT_TO_RIGHT):
        if mode == TEXT_FLOW_LEFT_TO_RIGHT:
            self._displayMode |= LCD_ENTRYLEFT
            self.sendCommand(LCD_ENTRYMODESET | self._displayMode)
        elif mode == TEXT_FLOW_RIGHT_TO_LEFT:
            self._displayMode &= ~LCD_ENTRYLEFT
            self.sendCommand(LCD_ENTRYMODESET | self._displayMode)
        else:
            raise ValueError("Invalid text flow direction")

    def setAutoScroll(self, auto=True):
        if auto is True:
            self._displayMode |= LCD_ENTRYSHIFTINCREMENT
        elif auto is False:
            self._displayMode &= ~LCD_ENTRYSHIFTINCREMENT
        else:
            raise ValueError("Invalid auto scrll option")