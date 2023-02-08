import adafruit_bme680
import time
import board
from datetime import datetime

now = datetime.now()

i2c = board.I2C()
bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c)
bme680.sea_level_pressure=1013.25
count_time = 0

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
