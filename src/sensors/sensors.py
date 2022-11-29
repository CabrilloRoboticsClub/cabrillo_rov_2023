'''
sensors.py

this ros2 node takes sensor data from i2c sensors and publishes it

Copyright (C) 2022-2023 Cabrillo Robotics Club

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

Cabrillo Robotics Club
6500 Soquel Drive Aptos, CA 95003
cabrillorobotics@gmail.com
'''

import board

from adafruit_bme280 import basic as adafruit_bme280

i2c = board.I2C()  # uses board.SCL and board.SDA

logic_tube_bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, 0x77)

thrust_box_bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, 0x76)


print("\nlogic tube")
print("\nTemperature: %0.1f C" % logic_tube_bme280.temperature)
print("Humidity: %0.1f %%" % logic_tube_bme280.humidity)
print("Pressure: %0.1f hPa" % logic_tube_bme280.pressure)


print("\nthrust box")
print("\nTemperature: %0.1f C" % thrust_box_bme280.temperature)
print("Humidity: %0.1f %%" % thrust_box_bme280.humidity)
print("Pressure: %0.1f hPa" % thrust_box_bme280.pressure)
