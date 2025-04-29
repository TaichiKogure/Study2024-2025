"""
Raspberry Pi Pico - Beginner Program 8: Reaction Time Game
=========================================================

This program creates a simple reaction time game where the player
must press a button as quickly as possible after an LED lights up.

Hardware required:
- Raspberry Pi Pico
- 1 LED
- 1 pushbutton
- 220 ohm resistor (for LED)
- 10K ohm resistor (for button pull-down)
- Breadboard and jumper wires

Connections:
- Connect the LED's anode (longer leg) to GPIO pin 15 through a 220 ohm resistor
- Connect the LED's cathode (shorter leg) to GND
- Connect one side of the button to 3.3V
- Connect the other side of the button to GPIO pin 16
- Connect a 10K resistor from GPIO pin 16 to GND (pull-down)

Instructions:
1. Set up the circuit as described above
2. Upload this program to the Pico
3. Open the serial console to view instructions
4. Follow the prompts to play the game

Concepts covered:
- Combining input and output
- Random number generation
- Measuring time intervals
- Creating interactive programs
- Game state management
"""

import machine
import time
import random

# Set up the LED and button
led = machine.Pin(15, machine.Pin.OUT)
button = machine.Pin(16, machine.Pin.IN)

# Initialize variables
best_time = float('inf')  # Initialize with infinity
game_active = False
start_time = 0

def display_instructions():
    print("\nReaction Time Game")
    print("=================")
    print("Test your reflexes!")
    print("1. Wait for the LED to light up")
    print("2. Press the button as fast as you can when it does")
    print("3. See your reaction time")
    print("\nPress the button to start a new round...")

try:
    display_instructions()
    
    while True:
        # Check if button is pressed to start a new round
        if not game_active and button.value() == 1:
            print("\nGet ready...")
            time.sleep(1)  # Give the player time to prepare
            
            # Random delay between 2 and 5 seconds
            delay = random.uniform(2, 5)
            time.sleep(delay)
            
            # Start the game
            led.value(1)  # Turn on the LED
            start_time = time.ticks_ms()  # Record the start time
            game_active = True
            print("GO! Press the button now!")
        
        # Check if button is pressed during active game
        elif game_active and button.value() == 1:
            # Calculate reaction time
            end_time = time.ticks_ms()
            reaction_time = time.ticks_diff(end_time, start_time) / 1000.0  # Convert to seconds
            
            # Turn off the LED
            led.value(0)
            
            # Update best time
            if reaction_time < best_time:
                best_time = reaction_time
                print(f"New best time: {best_time:.3f} seconds!")
            else:
                print(f"Your reaction time: {reaction_time:.3f} seconds (Best: {best_time:.3f})")
            
            # Reset game state
            game_active = False
            time.sleep(0.5)  # Debounce delay
            print("\nPress the button to play again...")
        
        # Small delay to prevent CPU hogging
        time.sleep(0.01)
            
except KeyboardInterrupt:
    # Clean up on exit
    led.value(0)
    print("\nGame over! Thanks for playing!")