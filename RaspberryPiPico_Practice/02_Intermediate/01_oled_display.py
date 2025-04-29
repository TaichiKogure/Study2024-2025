"""
Raspberry Pi Pico - Intermediate Program 1: OLED Display
========================================================

This program demonstrates how to interface with an OLED display
using the I2C communication protocol.

Hardware required:
- Raspberry Pi Pico
- SSD1306 OLED display (128x64 or 128x32 pixels)
- Breadboard and jumper wires

Connections:
- Connect OLED VCC to 3.3V
- Connect OLED GND to GND
- Connect OLED SCL to GPIO pin 17 (I2C1 SCL)
- Connect OLED SDA to GPIO pin 16 (I2C1 SDA)

Note: You'll need to install the SSD1306 library. Upload the ssd1306.py
file to your Pico before running this program.

Instructions:
1. Set up the circuit as described above
2. Upload the ssd1306.py library to your Pico
3. Upload this program to the Pico
4. The OLED display should show text and graphics

Concepts covered:
- I2C communication protocol
- Working with display libraries
- Drawing text and graphics
- Screen updates and refresh
"""

import machine
import time
import framebuf

# Import the SSD1306 library
# Note: You need to upload ssd1306.py to your Pico first
try:
    import ssd1306
except ImportError:
    print("Error: ssd1306 library not found")
    print("Please upload ssd1306.py to your Pico")
    raise

# Set up I2C
i2c = machine.I2C(1, sda=machine.Pin(16), scl=machine.Pin(17), freq=400000)

# Scan for I2C devices
devices = i2c.scan()
if devices:
    print(f"I2C devices found: {devices}")
else:
    print("No I2C devices found. Check your connections.")
    raise RuntimeError("No I2C devices detected")

# Initialize the OLED display
# Most common SSD1306 OLED displays are 128x64 or 128x32 pixels
# Adjust the height parameter (64 or 32) to match your display
oled_width = 128
oled_height = 64  # Change to 32 if you have a 128x32 display
oled = ssd1306.SSD1306_I2C(oled_width, oled_height, i2c)

def display_text(text, x, y, clear_first=True):
    """Display text at the specified position"""
    if clear_first:
        oled.fill(0)  # Clear the display (fill with black)
    oled.text(text, x, y)
    oled.show()

def draw_rectangle(x, y, width, height):
    """Draw a rectangle outline"""
    oled.rect(x, y, width, height, 1)
    oled.show()

def draw_filled_rectangle(x, y, width, height):
    """Draw a filled rectangle"""
    oled.fill_rect(x, y, width, height, 1)
    oled.show()

def draw_circle(x0, y0, radius):
    """Draw a circle using the Bresenham algorithm"""
    x = radius
    y = 0
    err = 0
    
    while x >= y:
        oled.pixel(x0 + x, y0 + y, 1)
        oled.pixel(x0 + y, y0 + x, 1)
        oled.pixel(x0 - y, y0 + x, 1)
        oled.pixel(x0 - x, y0 + y, 1)
        oled.pixel(x0 - x, y0 - y, 1)
        oled.pixel(x0 - y, y0 - x, 1)
        oled.pixel(x0 + y, y0 - x, 1)
        oled.pixel(x0 + x, y0 - y, 1)
        
        y += 1
        if err <= 0:
            err += 2 * y + 1
        if err > 0:
            x -= 1
            err -= 2 * x + 1
    
    oled.show()

try:
    # Display welcome message
    display_text("Raspberry Pi", 0, 0)
    display_text("Pico + OLED", 0, 10)
    display_text("I2C Demo", 0, 20)
    time.sleep(2)
    
    # Draw some shapes
    oled.fill(0)  # Clear the display
    draw_rectangle(10, 10, 108, 44)
    time.sleep(1)
    
    draw_filled_rectangle(20, 20, 88, 24)
    time.sleep(1)
    
    oled.fill(0)  # Clear the display
    draw_circle(64, 32, 20)
    time.sleep(1)
    
    # Display a counter
    for i in range(10):
        display_text(f"Counter: {i}", 0, 0)
        time.sleep(0.5)
    
    # Final message
    display_text("OLED Demo", 0, 0)
    display_text("Complete!", 0, 10)
    
except KeyboardInterrupt:
    # Clean up on exit
    oled.fill(0)
    oled.show()
    print("\nProgram stopped")