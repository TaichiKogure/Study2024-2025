"""
Raspberry Pi Pico - Advanced Program 4: Gesture Recognition
==========================================================

This program demonstrates how to implement a simple machine learning-based
gesture recognition system using an accelerometer to detect hand movements.

Hardware required:
- Raspberry Pi Pico
- MPU6050 accelerometer/gyroscope module
- OLED display (SSD1306, 128x64 pixels)
- RGB LED or 3 separate LEDs
- Pushbutton
- 10K ohm resistor (for button pull-down)
- 3 x 220 ohm resistors (for LEDs)
- Breadboard and jumper wires

Connections:
- MPU6050:
  - Connect VCC to 3.3V
  - Connect GND to GND
  - Connect SCL to GPIO pin 17 (I2C1 SCL)
  - Connect SDA to GPIO pin 16 (I2C1 SDA)

- OLED Display:
  - Connect VCC to 3.3V
  - Connect GND to GND
  - Connect SCL to GPIO pin 17 (I2C1 SCL)
  - Connect SDA to GPIO pin 16 (I2C1 SDA)

- RGB LED (or separate LEDs):
  - Connect Red LED to GPIO pin 2 through a 220 ohm resistor
  - Connect Green LED to GPIO pin 3 through a 220 ohm resistor
  - Connect Blue LED to GPIO pin 4 through a 220 ohm resistor
  - Connect common cathode to GND (or separate cathodes for separate LEDs)

- Pushbutton:
  - Connect one side to 3.3V
  - Connect the other side to GPIO pin 15 and to GND via 10K resistor

Note: You'll need to install the following libraries:
- mpu6050.py for the accelerometer
- ssd1306.py for the OLED display

Instructions:
1. Set up the circuit as described above
2. Upload the required libraries to your Pico
3. Upload this program to the Pico
4. Press the button to start training a gesture
5. Perform the gesture when prompted
6. Repeat for different gestures
7. After training, the system will recognize your gestures

Concepts covered:
- I2C communication with multiple devices
- Accelerometer data processing
- Simple machine learning (k-Nearest Neighbors algorithm)
- Gesture recognition techniques
- Training and classification phases
- User interface with OLED display
"""

import machine
import time
import math
import utime
import gc

# Import required libraries
try:
    import mpu6050
    import ssd1306
except ImportError:
    print("Error: Required libraries not found")
    print("Please upload mpu6050.py and ssd1306.py to your Pico")
    raise

# Set up I2C for the MPU6050 and OLED display
i2c = machine.I2C(1, sda=machine.Pin(16), scl=machine.Pin(17), freq=400000)

# Scan for I2C devices
devices = i2c.scan()
if devices:
    print(f"I2C devices found: {devices}")
else:
    print("No I2C devices found. Check your connections.")
    raise RuntimeError("No I2C devices detected")

# Set up the MPU6050
try:
    accelerometer = mpu6050.MPU6050(i2c)
    print("MPU6050 initialized")
except Exception as e:
    print(f"Error initializing MPU6050: {e}")
    raise

# Set up the OLED display
try:
    oled_width = 128
    oled_height = 64
    oled = ssd1306.SSD1306_I2C(oled_width, oled_height, i2c)
    print("OLED display initialized")
except Exception as e:
    print(f"Error initializing OLED display: {e}")
    raise

# Set up the RGB LED pins
red_led = machine.Pin(2, machine.Pin.OUT)
green_led = machine.Pin(3, machine.Pin.OUT)
blue_led = machine.Pin(4, machine.Pin.OUT)

# Set up the button
button = machine.Pin(15, machine.Pin.IN)

# Constants
SAMPLE_SIZE = 50  # Number of samples to collect for each gesture
NUM_FEATURES = 6  # 3 accelerometer axes + 3 gyroscope axes
NUM_GESTURES = 3  # Number of gestures to recognize
GESTURE_NAMES = ["Left-Right", "Up-Down", "Circle"]

# Global variables
gesture_data = [[] for _ in range(NUM_GESTURES)]  # Training data for each gesture
current_state = "IDLE"  # Current state of the system
current_gesture = 0     # Current gesture being trained or recognized
last_button_time = 0    # For debouncing

# Function to set RGB LED color
def set_led_color(r, g, b):
    red_led.value(r)
    green_led.value(g)
    blue_led.value(b)

# Function to clear the OLED display
def clear_display():
    oled.fill(0)
    oled.show()

# Function to display text on the OLED
def display_text(text_lines, clear_first=True):
    if clear_first:
        oled.fill(0)
    
    y = 0
    for line in text_lines:
        oled.text(line, 0, y)
        y += 10
    
    oled.show()

# Function to read accelerometer and gyroscope data
def read_sensor_data():
    try:
        accel = accelerometer.get_accel_data()
        gyro = accelerometer.get_gyro_data()
        
        # Extract the values
        ax, ay, az = accel['x'], accel['y'], accel['z']
        gx, gy, gz = gyro['x'], gyro['y'], gyro['z']
        
        return [ax, ay, az, gx, gy, gz]
    except Exception as e:
        print(f"Error reading sensor data: {e}")
        return [0, 0, 0, 0, 0, 0]

# Function to collect samples for a gesture
def collect_gesture_samples(gesture_index):
    samples = []
    
    # Display instructions
    display_text([
        f"Gesture: {GESTURE_NAMES[gesture_index]}",
        "Press button",
        "to start recording",
        "the gesture"
    ])
    
    # Wait for button press to start
    while button.value() == 0:
        time.sleep(0.1)
    
    # Debounce
    time.sleep(0.2)
    
    # Display countdown
    for i in range(3, 0, -1):
        display_text([
            f"Gesture: {GESTURE_NAMES[gesture_index]}",
            f"Starting in {i}...",
            "Get ready!"
        ])
        time.sleep(1)
    
    # Start collecting samples
    display_text([
        f"Gesture: {GESTURE_NAMES[gesture_index]}",
        "RECORDING NOW!",
        "Perform the gesture"
    ])
    
    # Set LED to blue during recording
    set_led_color(0, 0, 1)
    
    # Collect samples
    for _ in range(SAMPLE_SIZE):
        sample = read_sensor_data()
        samples.append(sample)
        time.sleep(0.02)  # 50 samples per second
    
    # Set LED off after recording
    set_led_color(0, 0, 0)
    
    # Display completion
    display_text([
        f"Gesture: {GESTURE_NAMES[gesture_index]}",
        "Recording complete!",
        f"{SAMPLE_SIZE} samples collected"
    ])
    
    time.sleep(1)
    
    return samples

# Function to calculate Euclidean distance between two feature vectors
def euclidean_distance(vector1, vector2):
    if len(vector1) != len(vector2):
        return float('inf')
    
    sum_squared_diff = 0
    for i in range(len(vector1)):
        sum_squared_diff += (vector1[i] - vector2[i]) ** 2
    
    return math.sqrt(sum_squared_diff)

# Function to normalize a list of samples (simple scaling)
def normalize_samples(samples):
    # Find min and max for each feature
    min_values = [float('inf')] * NUM_FEATURES
    max_values = [float('-inf')] * NUM_FEATURES
    
    for sample in samples:
        for i in range(NUM_FEATURES):
            min_values[i] = min(min_values[i], sample[i])
            max_values[i] = max(max_values[i], sample[i])
    
    # Normalize each sample
    normalized_samples = []
    for sample in samples:
        normalized_sample = []
        for i in range(NUM_FEATURES):
            # Avoid division by zero
            if max_values[i] == min_values[i]:
                normalized_value = 0
            else:
                normalized_value = (sample[i] - min_values[i]) / (max_values[i] - min_values[i])
            normalized_sample.append(normalized_value)
        normalized_samples.append(normalized_sample)
    
    return normalized_samples

# Function to train the gesture recognition system
def train_gestures():
    global gesture_data
    
    display_text([
        "Gesture Training",
        "----------------",
        "Let's train the",
        "recognition system"
    ])
    
    time.sleep(2)
    
    # Train each gesture
    for i in range(NUM_GESTURES):
        # Collect samples for this gesture
        samples = collect_gesture_samples(i)
        
        # Normalize the samples
        normalized_samples = normalize_samples(samples)
        
        # Store the normalized samples
        gesture_data[i] = normalized_samples
        
        # Free up memory
        gc.collect()
    
    # Display completion
    display_text([
        "Training Complete!",
        "----------------",
        "All gestures trained",
        "Ready to recognize"
    ])
    
    # Set LED to green to indicate training complete
    set_led_color(0, 1, 0)
    time.sleep(2)
    set_led_color(0, 0, 0)

# Function to recognize a gesture using k-Nearest Neighbors algorithm
def recognize_gesture():
    # Display instructions
    display_text([
        "Gesture Recognition",
        "----------------",
        "Press button",
        "to perform a gesture"
    ])
    
    # Wait for button press to start
    while button.value() == 0:
        time.sleep(0.1)
    
    # Debounce
    time.sleep(0.2)
    
    # Display countdown
    for i in range(3, 0, -1):
        display_text([
            "Gesture Recognition",
            f"Starting in {i}...",
            "Get ready!"
        ])
        time.sleep(1)
    
    # Start collecting samples
    display_text([
        "Gesture Recognition",
        "RECORDING NOW!",
        "Perform the gesture"
    ])
    
    # Set LED to blue during recording
    set_led_color(0, 0, 1)
    
    # Collect samples
    test_samples = []
    for _ in range(SAMPLE_SIZE):
        sample = read_sensor_data()
        test_samples.append(sample)
        time.sleep(0.02)  # 50 samples per second
    
    # Set LED off after recording
    set_led_color(0, 0, 0)
    
    # Normalize the test samples
    normalized_test_samples = normalize_samples(test_samples)
    
    # Calculate the average distance to each gesture's samples
    distances = []
    for gesture_index in range(NUM_GESTURES):
        gesture_distances = []
        for test_sample in normalized_test_samples:
            for train_sample in gesture_data[gesture_index]:
                distance = euclidean_distance(test_sample, train_sample)
                gesture_distances.append(distance)
        
        # Calculate average distance
        avg_distance = sum(gesture_distances) / len(gesture_distances) if gesture_distances else float('inf')
        distances.append(avg_distance)
    
    # Find the gesture with the minimum average distance
    recognized_gesture = distances.index(min(distances))
    
    # Display the result
    display_text([
        "Recognized Gesture:",
        f"{GESTURE_NAMES[recognized_gesture]}",
        "",
        "Press button to try again"
    ])
    
    # Set LED color based on recognized gesture
    if recognized_gesture == 0:
        set_led_color(1, 0, 0)  # Red for Left-Right
    elif recognized_gesture == 1:
        set_led_color(0, 1, 0)  # Green for Up-Down
    elif recognized_gesture == 2:
        set_led_color(0, 0, 1)  # Blue for Circle
    
    time.sleep(2)
    set_led_color(0, 0, 0)

# Function to check button press with debouncing
def check_button():
    global last_button_time
    
    if button.value() == 1:
        current_time = utime.ticks_ms()
        if utime.ticks_diff(current_time, last_button_time) > 300:
            last_button_time = current_time
            return True
    
    return False

# Main function
def main():
    global current_state
    
    try:
        # Initialize
        clear_display()
        set_led_color(0, 0, 0)
        
        # Display welcome message
        display_text([
            "Gesture Recognition",
            "----------------",
            "Press button to",
            "start training"
        ])
        
        while True:
            if current_state == "IDLE":
                if check_button():
                    current_state = "TRAINING"
            
            elif current_state == "TRAINING":
                train_gestures()
                current_state = "RECOGNITION"
            
            elif current_state == "RECOGNITION":
                recognize_gesture()
                # Stay in recognition state
            
            # Small delay to prevent CPU hogging
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
    finally:
        # Clean up
        clear_display()
        set_led_color(0, 0, 0)
        print("Gesture recognition system shutdown complete")

# Run the main function
if __name__ == "__main__":
    main()