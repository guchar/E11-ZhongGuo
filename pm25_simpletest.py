# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""
Example sketch to connect to PM2.5 sensor with either I2C or UART.
"""

# pylint: disable=unused-import
import time
import board
import busio
from digitalio import DigitalInOut, Direction, Pull
from adafruit_pm25.i2c import PM25_I2C
import adafruit_bme680 
import csv
from datetime import datetime
import argparse 
import sys



reset_pin = None
# If you have a GPIO, its not a bad idea to connect it to the RESET pin
# reset_pin = DigitalInOut(board.G0)
# reset_pin.direction = Direction.OUTPUT
# reset_pin.value = False


# For use with a computer running Windows:
# import serial
# uart = serial.Serial("COM30", baudrate=9600, timeout=1)

# For use with microcontroller board:
# (Connect the sensor TX pin to the board/computer RX pin)
# uart = busio.UART(board.TX, board.RX, baudrate=9600)

# For use with Raspberry Pi/Linux:
import serial
uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=0.95)

# For use with USB-to-serial cable:
# import serial
# uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=0.25)

# Connect to a PM2.5 sensor over UART
from adafruit_pm25.uart import PM25_UART
pm25 = PM25_UART(uart, reset_pin)

# Create library object, use 'slow' 100KHz frequency!
# i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
# Connect to a PM2.5 sensor over I2C
# pm25 = PM25_I2C(i2c, reset_pin)

print("Found PM2.5 sensor, reading data...")

file = open("data.csv", "w")
writer = csv.writer(file)

start_time = time.time()
run_time = int(sys.argv[1])
itime = start_time

i2c = board.I2C()
bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c)
bme680.sea_level_pressure=1013.25
count_time = 0

metadata = ["Timestamp", "PM 1.0", "PM 2.5", "PM 10", "Temperature", "Gas", "Humidity", "Pressure", "Altitude"]

writer.writerow(metadata)

while itime < (start_time + run_time):
    time.sleep(1)
    current_time = time.time()
    myobj = datetime.now()

    
    # writer.writerow("Current timestamp: ") 
    # writer.writerow(["{}:{}:{}".format(myobj.hour, myobj.minute, myobj.second)])
    # writer.writerow(["Concentration Units (standard)"])
    # writer.writerow("---------------------------------------")
    try:
        aqdata = pm25.read()
        # print(aqdata)
    except RuntimeError:
        print("Unable to read from sensor, retrying...")
        continue

    data = [current_time, aqdata["pm10 standard"], aqdata["pm25 standard"], aqdata["pm100 standard"], bme680.temperature, bme680.gas, bme680.relative_humidity, bme680.pressure, bme680.altitude]


    writer.writerow(data)

    # writer.writerow("PM 1.0: %d\tPM2.5: %d\tPM10: %d"
    #     % (aqdata["pm10 standard"], aqdata["pm25 standard"], aqdata["pm100 standard"]))


    print("Current timestamp -- {}:{}:{}".format(myobj.hour, myobj.minute, myobj.second))
    print()
    print("Concentration Units (standard)")
    print("---------------------------------------")
    print(
        "PM 1.0: %d\tPM2.5: %d\tPM10: %d"
        % (aqdata["pm10 standard"], aqdata["pm25 standard"], aqdata["pm100 standard"])
    )
    print("Concentration Units (environmental)")
    print("---------------------------------------")
    print(
        "PM 1.0: %d\tPM2.5: %d\tPM10: %d"
        % (aqdata["pm10 env"], aqdata["pm25 env"], aqdata["pm100 env"])
    )
    print("---------------------------------------")
    print("Particles > 0.3um / 0.1L air:", aqdata["particles 03um"])
    print("Particles > 0.5um / 0.1L air:", aqdata["particles 05um"])
    print("Particles > 1.0um / 0.1L air:", aqdata["particles 10um"])
    print("Particles > 2.5um / 0.1L air:", aqdata["particles 25um"])
    print("Particles > 5.0um / 0.1L air:", aqdata["particles 50um"])
    print("Particles > 10 um / 0.1L air:", aqdata["particles 100um"])
    print("---------------------------------------")

    print("")

now = datetime.now()



while count_time < 10:
	now = datetime.now()
	current_time = now.strftime("%H:%M:%S")
	print("")
	print("Current Time =", current_time, "Temperature: %0.1f C" % bme680.temperature, 'Gas: %d ohm' % bme680.gas, 'Humidity: %0.1f %%' % bme680.relative_humidity, 'Pressure: %0.3f hPa' % bme680.pressure, 'Altitude = %0.2f meters' % bme680.altitude)
	'''print("Temperature: %0.1f C" % bme680.temperature)
	print('Gas: %d ohm' % bme680.gas)
	print('Humidity: %0.1f %%' % bme680.relative_humidity)
	print('Pressure: %0.3f hPa' % bme680.pressure)
	print('Altitude = %0.2f meters' % bme680.altitude)'''
	count_time += 1
	
	time.sleep(2)

file.close() 