import csv
import time
import matplotlib.pyplot as plt
import matplotlib.dates as md
import mh_z19
import datetime

import os

os.system('sudo lsof /dev/serial0 | grep python | awk \'{print "kill -9", $2}\' | sh')

def read():
    out = mh_z19.read
    return str(out)[8:-1]


if __name__ == '__main__':
    fig, ax = plt.subplots()
    ax.xaxis.set_major_formatter(md.DateFormatter('%H:%M'))

    while True:
        co2 = read()
        print(co2)

        current_time = datetime.datetime.now()

        # Append the data to the csv file
        with open('data.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([current_time, co2])

        # Read the updated csv file
        data = []
        with open('data.csv', 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                data.append(row)

        # Split the data into time and co2 for plotting
        times = [datetime.datetime.strptime(row[0], '%Y-%m-%d %H:%M:%S.%f') for row in data]
        co2_values = [float(row[1]) for row in data]

        # Clear the previous plot
        plt.cla()

        # Plot the updated data
        ax.plot(times, co2_values)
        fig.autofmt_xdate()
        plt.draw()

        # Pause for 60 seconds
        plt.pause(60)

#!/usr/bin/env python

import bme680
print("""temperature-pressure-humidity.py - Displays temperature, pressure, and humidity.

If you don't need gas readings, then you can read temperature,
pressure and humidity quickly.

Press Ctrl+C to exit

""")

try:
    sensor = bme680.BME680(bme680.I2C_ADDR_PRIMARY)
except (RuntimeError, IOError):
    sensor = bme680.BME680(bme680.I2C_ADDR_SECONDARY)

# These oversampling settings can be tweaked to
# change the balance between accuracy and noise in
# the data.

sensor.set_humidity_oversample(bme680.OS_2X)
sensor.set_pressure_oversample(bme680.OS_4X)
sensor.set_temperature_oversample(bme680.OS_8X)
sensor.set_filter(bme680.FILTER_SIZE_3)

print('Polling:')
try:
    while True:
        if sensor.get_sensor_data():
            output = '{0:.2f} C,{1:.2f} hPa,{2:.3f} %RH'.format(
                sensor.data.temperature,
                sensor.data.pressure,
                sensor.data.humidity)
            print(output)

except KeyboardInterrupt:
    pass

#!/usr/bin/env python

import time

import bme680

print("""indoor-air-quality.py - Estimates indoor air quality.

Runs the sensor for a burn-in period, then uses a
combination of relative humidity and gas resistance
to estimate indoor air quality as a percentage.

Press Ctrl+C to exit!

""")

try:
    sensor = bme680.BME680(bme680.I2C_ADDR_PRIMARY)
except (RuntimeError, IOError):
    sensor = bme680.BME680(bme680.I2C_ADDR_SECONDARY)

# These oversampling settings can be tweaked to
# change the balance between accuracy and noise in
# the data.

sensor.set_humidity_oversample(bme680.OS_2X)
sensor.set_pressure_oversample(bme680.OS_4X)
sensor.set_temperature_oversample(bme680.OS_8X)
sensor.set_filter(bme680.FILTER_SIZE_3)
sensor.set_gas_status(bme680.ENABLE_GAS_MEAS)

sensor.set_gas_heater_temperature(320)
sensor.set_gas_heater_duration(150)
sensor.select_gas_heater_profile(0)

# start_time and curr_time ensure that the
# burn_in_time (in seconds) is kept track of.

start_time = time.time()
curr_time = time.time()
burn_in_time = 300

burn_in_data = []

try:
    # Collect gas resistance burn-in values, then use the average
    # of the last 50 values to set the upper limit for calculating
    # gas_baseline.
    print('Collecting gas resistance burn-in data for 5 mins\n')
    while curr_time - start_time < burn_in_time:
        curr_time = time.time()
        if sensor.get_sensor_data() and sensor.data.heat_stable:
            gas = sensor.data.gas_resistance
            burn_in_data.append(gas)
            print('Gas: {0} Ohms'.format(gas))
            time.sleep(1)

    gas_baseline = sum(burn_in_data[-50:]) / 50.0

    # Set the humidity baseline to 40%, an optimal indoor humidity.
    hum_baseline = 40.0

    # This sets the balance between humidity and gas reading in the
    # calculation of air_quality_score (25:75, humidity:gas)
    hum_weighting = 0.25

    print('Gas baseline: {0} Ohms, humidity baseline: {1:.2f} %RH\n'.format(
        gas_baseline,
        hum_baseline))

    while True:
        if sensor.get_sensor_data() and sensor.data.heat_stable:
            gas = sensor.data.gas_resistance
            gas_offset = gas_baseline - gas

            hum = sensor.data.humidity
            hum_offset = hum - hum_baseline

            # Calculate hum_score as the distance from the hum_baseline.
            if hum_offset > 0:
                hum_score = (100 - hum_baseline - hum_offset)
                hum_score /= (100 - hum_baseline)
                hum_score *= (hum_weighting * 100)

            else:
                hum_score = (hum_baseline + hum_offset)
                hum_score /= hum_baseline
                hum_score *= (hum_weighting * 100)

            # Calculate gas_score as the distance from the gas_baseline.
            if gas_offset > 0:
                gas_score = (gas / gas_baseline)
                gas_score *= (100 - (hum_weighting * 100))

            else:
                gas_score = 100 - (hum_weighting * 100)

            # Calculate air_quality_score.
            air_quality_score = hum_score + gas_score

            print('Gas: {0:.2f} Ohms,humidity: {1:.2f} %RH,air quality: {2:.2f}'.format(
                gas,
                hum,
                air_quality_score))

            time.sleep(1)

except KeyboardInterrupt:
    pass
