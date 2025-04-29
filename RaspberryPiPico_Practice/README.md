# Raspberry Pi Pico Practice Programs

This repository contains a collection of 20 practice programs for the Raspberry Pi Pico microcontroller, organized by difficulty level from beginner to advanced. These programs are designed to help you learn MicroPython programming on the Pico and explore various hardware interfacing techniques.

## Repository Structure

The programs are organized into three difficulty levels:

```
RaspberryPiPico_Practice/
├── 01_Beginner/       # Basic I/O operations and simple projects
├── 02_Intermediate/   # More complex sensors and communication protocols
└── 03_Advanced/       # Complete projects with multiple components and advanced concepts
```

## Requirements

- Raspberry Pi Pico or Pico W
- MicroPython firmware installed on the Pico
- Various components as specified in each program
- USB cable for programming and power
- Computer with Thonny IDE or another MicroPython development environment

## Installation

1. Install MicroPython on your Raspberry Pi Pico if you haven't already:
   - Download the latest MicroPython firmware from the [official website](https://micropython.org/download/rp2-pico/)
   - Hold the BOOTSEL button on the Pico while connecting it to your computer
   - Drag and drop the firmware file onto the RPI-RP2 drive that appears

2. Clone or download this repository to your computer

3. Use Thonny IDE or another tool to upload the programs to your Pico:
   - Open the program file in Thonny
   - Select "Raspberry Pi Pico" as the interpreter
   - Click "Save as..." and save to the Pico

4. Some programs require additional libraries. Upload the necessary library files to your Pico before running these programs.

## Program List

### Beginner Programs

1. **Blinking LED** - Control the onboard LED with basic GPIO operations
2. **Button Input** - Read input from a button and respond to it
3. **PWM LED Fade** - Create a breathing effect with an LED using PWM
4. **Traffic Light** - Simulate a traffic light sequence with multiple LEDs
5. **Potentiometer LED** - Control LED brightness with a potentiometer
6. **Piezo Melody** - Play simple melodies using a piezo buzzer
7. **Temperature Monitor** - Read the Pico's built-in temperature sensor
8. **Reaction Game** - Test your reflexes with a simple button-pressing game

### Intermediate Programs

1. **OLED Display** - Interface with an SSD1306 OLED display using I2C
2. **DHT Sensor** - Read temperature and humidity from a DHT22/DHT11 sensor
3. **Ultrasonic Sensor** - Measure distances with an HC-SR04 ultrasonic sensor
4. **Servo Control** - Control servo motors with different input methods
5. **RFID Reader** - Interface with an MFRC522 RFID reader using SPI
6. **Stepper Motor** - Control a stepper motor with different stepping modes

### Advanced Programs

1. **Data Logger** - Record sensor readings to a file with timestamps
2. **WiFi Webserver** - Create a web server for remote monitoring and control (Pico W only)
3. **Multitasking Robot** - Use cooperative multitasking to control a robot
4. **Gesture Recognition** - Implement a simple machine learning system for gesture recognition
5. **BLE Smart Home** - Create a Bluetooth Low Energy smart home controller (Pico W only)
6. **Balancing Robot** - Implement a PID control algorithm for a self-balancing robot

## Usage

Each program includes detailed comments explaining:
- Hardware requirements
- Connection instructions
- Program functionality
- Concepts covered

Before running a program, make sure to:
1. Set up the circuit as described in the program's comments
2. Upload any required libraries to your Pico
3. Upload the program to your Pico
4. Follow the specific instructions for each program

## Required Libraries

Some programs require additional libraries. Here are the main ones:

- `dht.py` - For DHT temperature and humidity sensors
- `ssd1306.py` - For OLED displays
- `mpu6050.py` - For the MPU6050 accelerometer/gyroscope
- `mfrc522.py` - For RFID readers
- `sdcard.py` - For SD card modules
- `ds3231.py` - For RTC modules
- `bluetooth.py` and `aioble.py` - For BLE functionality (Pico W only)

You can find these libraries in the MicroPython package index or from their respective GitHub repositories.

## Notes for Specific Programs

- **WiFi Webserver** and **BLE Smart Home** require a Raspberry Pi Pico W (with wireless capabilities)
- **Data Logger** requires a microSD card module and an RTC module
- **Balancing Robot** requires careful mechanical assembly and PID tuning
- Some programs may need parameter adjustments based on your specific hardware

## Contributing

Feel free to contribute to this repository by:
- Improving existing programs
- Adding new programs
- Fixing bugs or documentation
- Suggesting enhancements

## License

This project is released under the MIT License. See the LICENSE file for details.

## Acknowledgments

- The Raspberry Pi Foundation for creating the Pico
- The MicroPython community for their excellent documentation and libraries
- All contributors to the open-source libraries used in these programs