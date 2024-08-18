from gpiozero import Button, MCP3008
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from time import sleep
import math
import csv
import threading
import datetime
import os

# Load kernel modules
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

# ID setting
makerobo_DS18b20 = '28-00000084826b'

# GPIO settings
makerobo_DO = Button(17)
makerobo_tempPIN = MCP3008(channel=0)


def makerobo_setup():
    global makerobo_DS18b20
    for i in os.listdir('/sys/bus/w1/devices'):
        if i != 'wl_bus_master1':
            makerobo_DS18b20 = i


def makerobo_read():
    makerobo_location = '/sys/bus/w1/devices/' + makerobo_DS18b20 + '/w1_slave'
    try:
        with open(makerobo_location, 'r') as makerobo_tfile:
            makerobo_text = makerobo_tfile.read()
        secondline = makerobo_text.split('\n')[1]
        temperaturedata = secondline.split(" ")[9]
        temperature = float(temperaturedata[2:]) / 1000.0
        return temperature
    except (IndexError, FileNotFoundError) as e:
        return None


# Initialize lists to store data
x = []
y1 = []
y2 = []
csvfile = "Pi5_EnvData.csv"

# Open CSV file for writing
csv_file = open(csvfile, "w")
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["datetime", "temp_C_sensor1", "temp_C_sensor2"])


def makerobo_loop(runtime):
    start_time = datetime.datetime.now()
    makerobo_Status = 1

    while True:
        current_datetime = datetime.datetime.now()
        if current_datetime - start_time > datetime.timedelta(minutes=runtime):
            makerobo_DO.close()
            makerobo_tempPIN.close()
            break

        makerobo_analogVal = makerobo_tempPIN.value
        makerobo_Vr = float(makerobo_analogVal) * 3.3
        makerobo_Rt = 10000 * makerobo_Vr / (3.3 - makerobo_Vr)
        makerobo_Temp = 1 / (((math.log(makerobo_Rt / 10000)) / 3950) + (1 / (273.15 + 25)))
        temp_c = makerobo_Temp - 273.15
        temp_f = temp_c * 9.0 / 5.0 + 32
        lcd_datetime = current_datetime.strftime("%Y-%m-%d %H:%M:%S")
        print(f"DateTime = {lcd_datetime}, Temp C = {temp_c:.2f}, Temp F = {temp_f:.2f}")

        temperature = makerobo_read()
        if temperature is None:
            continue

        makerobo_tmp = not makerobo_DO.is_pressed
        if makerobo_tmp != makerobo_Status:
            makerobo_Status = makerobo_tmp

        if temperature is not None:
            print(f"Current temperature on Sensor2 : {temperature:.3f} C")

        sleep(10)

        x.append(current_datetime)
        y1.append(temp_c)
        y2.append(makerobo_read())

        # Write data to CSV immediately
        csv_writer.writerow([lcd_datetime, y1[-1], y2[-1]])
        csv_file.flush()


def destroy():
    csv_file.close()


if __name__ == '__main__':
    try:
        makerobo_setup()
        makerobo_loop(60 * 24 * 365)
    except KeyboardInterrupt:
        pass
    finally:
        destroy()

plt.show()
