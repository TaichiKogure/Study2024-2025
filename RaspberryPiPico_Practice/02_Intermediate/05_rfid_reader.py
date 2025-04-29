"""
Raspberry Pi Pico - Intermediate Program 5: RFID Card Reader
===========================================================

This program demonstrates how to interface with an MFRC522 RFID reader
module using SPI communication to read RFID/NFC cards and tags.

Hardware required:
- Raspberry Pi Pico
- MFRC522 RFID reader module
- RFID cards or tags (13.56 MHz)
- LED (optional, for indicating successful reads)
- 220 ohm resistor (for LED)
- Breadboard and jumper wires

Connections:
- Connect MFRC522 VCC to 3.3V
- Connect MFRC522 GND to GND
- Connect MFRC522 RST to GPIO pin 20
- Connect MFRC522 MISO to GPIO pin 16 (SPI0 RX)
- Connect MFRC522 MOSI to GPIO pin 19 (SPI0 TX)
- Connect MFRC522 SCK to GPIO pin 18 (SPI0 SCK)
- Connect MFRC522 SDA/CS to GPIO pin 17 (SPI0 CSn)
- Optional: Connect LED anode to GPIO pin 15 through a 220 ohm resistor
- Optional: Connect LED cathode to GND

Note: You'll need to install the mfrc522 library. Upload the mfrc522.py
file to your Pico before running this program.

Instructions:
1. Set up the circuit as described above
2. Upload the mfrc522.py library to your Pico
3. Upload this program to the Pico
4. Open the serial console to view the output
5. Present an RFID card or tag to the reader

Concepts covered:
- SPI communication protocol
- RFID/NFC technology basics
- Working with external libraries
- Card ID processing and validation
- Access control concepts
"""

import machine
import utime
import time

# Import the MFRC522 library
# Note: You need to upload mfrc522.py to your Pico first
try:
    import mfrc522
except ImportError:
    print("Error: mfrc522 library not found")
    print("Please upload mfrc522.py to your Pico")
    raise

# Set up the LED pin (optional)
led = machine.Pin(15, machine.Pin.OUT)

# Set up SPI for the RFID reader
spi = machine.SPI(0,
                 baudrate=100000,
                 polarity=0,
                 phase=0,
                 bits=8,
                 firstbit=machine.SPI.MSB,
                 sck=machine.Pin(18),
                 mosi=machine.Pin(19),
                 miso=machine.Pin(16))

# Set up the RFID reader
sda = machine.Pin(17, machine.Pin.OUT)
rst = machine.Pin(20, machine.Pin.OUT)
rfid = mfrc522.MFRC522(spi, sda, rst)

# List of authorized card IDs (replace with your card IDs)
# Format: [id1, id2, id3, id4]
authorized_cards = [
    [211, 237, 184, 117],  # Example card 1
    [131, 95, 43, 203]     # Example card 2
]

def uid_to_string(uid):
    """Convert the UID bytes to a formatted string"""
    return '-'.join([str(x) for x in uid])

def is_authorized(uid):
    """Check if the card UID is in the authorized list"""
    for card in authorized_cards:
        if uid[:len(card)] == card:
            return True
    return False

def blink_led(times=3, delay=0.1):
    """Blink the LED a specified number of times"""
    for _ in range(times):
        led.value(1)
        time.sleep(delay)
        led.value(0)
        time.sleep(delay)

try:
    print("MFRC522 RFID Reader Demo")
    print("========================")
    print("Waiting for RFID card...")
    
    # Track the last read card to prevent continuous readings of the same card
    last_card = None
    last_read_time = 0
    
    while True:
        # Check if a card is present
        (stat, tag_type) = rfid.request(rfid.REQIDL)
        
        if stat == rfid.OK:
            # Card detected, get the UID
            (stat, uid) = rfid.anticoll()
            
            if stat == rfid.OK:
                # Convert UID to string for display
                card_id = uid_to_string(uid)
                current_time = utime.ticks_ms()
                
                # Check if this is a new card or if enough time has passed
                if last_card != card_id or utime.ticks_diff(current_time, last_read_time) > 3000:
                    print(f"Card detected! UID: {card_id}")
                    
                    # Check if the card is authorized
                    if is_authorized(uid):
                        print("Access granted!")
                        blink_led(3, 0.1)  # Blink LED 3 times quickly
                    else:
                        print("Access denied!")
                        blink_led(1, 1.0)  # Blink LED once slowly
                    
                    # Update last card and time
                    last_card = card_id
                    last_read_time = current_time
                
                # Select the card
                rfid.select(uid)
                
                # For MIFARE Classic cards, you could authenticate and read data blocks here
                # This is a more advanced topic
        
        # Small delay before checking again
        time.sleep(0.1)
        
except KeyboardInterrupt:
    # Clean up on exit
    led.value(0)
    print("\nProgram stopped")