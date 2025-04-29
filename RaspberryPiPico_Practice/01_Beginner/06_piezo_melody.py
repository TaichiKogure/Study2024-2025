"""
Raspberry Pi Pico - Beginner Program 6: Piezo Buzzer Melody
==========================================================

This program demonstrates how to generate tones using a piezo buzzer
to play a simple melody.

Hardware required:
- Raspberry Pi Pico
- Piezo buzzer
- Breadboard and jumper wires

Connections:
- Connect one pin of the piezo buzzer to GPIO pin 15
- Connect the other pin of the piezo buzzer to GND

Instructions:
1. Set up the circuit as described above
2. Upload this program to the Pico
3. Listen to the melody

Concepts covered:
- Generating tones with PWM
- Working with musical frequencies
- Creating sequences with arrays/lists
- Timing control for musical notes
"""

import machine
import time

# Set up the buzzer pin for PWM
buzzer_pin = machine.Pin(15)
pwm = machine.PWM(buzzer_pin)

# Define musical notes frequencies (in Hz)
NOTE_C4 = 262
NOTE_D4 = 294
NOTE_E4 = 330
NOTE_F4 = 349
NOTE_G4 = 392
NOTE_A4 = 440
NOTE_B4 = 494
NOTE_C5 = 523

# Function to play a tone
def play_tone(frequency, duration):
    if frequency > 0:
        pwm.freq(frequency)
        pwm.duty_u16(32768)  # 50% duty cycle (half volume)
    else:
        pwm.duty_u16(0)  # Silence
    
    time.sleep(duration)
    pwm.duty_u16(0)  # Stop the tone
    time.sleep(0.05)  # Small pause between notes

# Define the melody - "Twinkle Twinkle Little Star"
# Each tuple contains (frequency, duration)
melody = [
    (NOTE_C4, 0.3), (NOTE_C4, 0.3), (NOTE_G4, 0.3), (NOTE_G4, 0.3),
    (NOTE_A4, 0.3), (NOTE_A4, 0.3), (NOTE_G4, 0.6),
    (NOTE_F4, 0.3), (NOTE_F4, 0.3), (NOTE_E4, 0.3), (NOTE_E4, 0.3),
    (NOTE_D4, 0.3), (NOTE_D4, 0.3), (NOTE_C4, 0.6)
]

try:
    print("Playing melody...")
    
    # Play the melody
    for note in melody:
        frequency, duration = note
        play_tone(frequency, duration)
    
    # Play a final note
    play_tone(NOTE_C4, 0.6)
    
    print("Melody finished!")
    
except KeyboardInterrupt:
    # Clean up on exit
    pwm.duty_u16(0)  # Turn off the buzzer
    print("Program stopped")