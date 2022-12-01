'''
i2c_proxy.py

this is the node with the loc on /dev/i2c
everything using the i2c bus goes through this node

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

# # # # # # # # 
#
# IMPORTS
#
# # # # # # # #

# this library is needed by the bno085
import time

# library for accessing the raspberry pi board
# aka /dev/i2c
import board

# import the bme280 circuit python sensor library
from adafruit_bme280 import basic.Adafruit_BME280_I2C as adafruit_bme280

# inport the bno085 circuit python sensor library
from adafruit_bno08x.i2c import BNO08X_I2C as adafruit_bno08x


# # # # # # # #
#
# INSTANCIATIONS
#
# # # # # # # #

# grab the i2c interface for us to use
i2c = board.i2c

# instanciate the logic tube bme280
# enviromental sensor
logic_tube_bme280 = adafruit_bme280(i2c, 0x77)

# instanciate the thrust box bme280
# enviromental sensor
thrust_box_bme280 = adafruit_bme280(i2c, 0x76)

# instanciate the logic tube bmo085
# 9dof absolute orientation imu Ssensor
logic_tube_imu = adafruit_bno08x(i2c)


# # # # # # # #
#
# SENSOR DATA GRAB CODE
#
# # # # # # # #

# logic tube enviroment
print("\nlogic tube")
print("\nTemperature: %0.1f C" % logic_tube_bme280.temperature)
print("Humidity: %0.1f %%" % logic_tube_bme280.humidity)
print("Pressure: %0.1f hPa" % logic_tube_bme280.pressure)

# thust box enviroment
print("\nthrust box")
print("\nTemperature: %0.1f C" % thrust_box_bme280.temperature)
print("Humidity: %0.1f %%" % thrust_box_bme280.humidity)
print("Pressure: %0.1f hPa" % thrust_box_bme280.pressure)

# logic tube imu
print("\n logic tube imu")
# insert snesor reading code here