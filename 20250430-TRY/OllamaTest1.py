import numpy as np
import matplotlib.pyplot as plt

def generate_sine_wave(t, amplitude, frequency):
    return amplitude * np.sin(2 * np.pi * frequency * t)

# Define some dummy parameters
amplitude_charge = 3
frequency_charge = 5

amplitude_discharge = 4.1
frequency_discharge = 2

n_points = 200  # The number of points for the curve

# Generate time array from 0 to 10 seconds
time_array = np.linspace(0, 10, n_points)

# Generate sine wave curves
voltage_charge = generate_sine_wave(time_array, amplitude_charge, frequency_charge)
capacity_charge = generate_sine_wave(time_array, amplitude_charge * 2, frequency_charge * 1.5)

voltage_discharge = generate_sine_wave(time_array, amplitude_discharge, frequency_discharge)
capacity_discharge = generate_sine_wave(time_array, amplitude_discharge * 2, frequency_discharge * 1.5)

# Plotting
plt.figure(figsize=(8, 6))
plt.plot(capacity_charge, voltage_charge, label='Charge', color='red')
plt.plot(capacity_discharge, voltage_discharge, label='Discharge', color='blue')
plt.xlabel('Capacity [mAh]')
plt.ylabel('Cell Voltage [V]')
plt.title('Dummy Charge/Discharge Curve')
plt.grid(True)
plt.legend()
plt.ylim(-6, 6)  # Setting the maximum y-limit to prevent overflow
plt.xlim(-10, 10)
plt.show()