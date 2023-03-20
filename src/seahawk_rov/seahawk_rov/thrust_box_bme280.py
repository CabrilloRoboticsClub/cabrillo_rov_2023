'''
seahawk_rov/thrust_box_bme280.py

code for publishing the data from the bme280 sensor in the thrust box

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

# time is needed
import time

# ros msgs
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import RelativeHumidity
from sensor_msgs.msg import FluidPressure

# import the bme280 circuit python sensor library
from adafruit_bme280 import basic as adafruit_bme280

class ThrustBoxBME280:
    def __init__(self, node, i2c):
        # create publishers
        self.temperature_publisher = node.create_publisher(Temperature,'thrust_box/temperature', 8)
        self.humidity_publisher = node.create_publisher(RelativeHumidity,'thrust_box/humidity', 8)
        self.pressure_publisher = node.create_publisher(FluidPressure,'thrust_box/pressure', 8)

        # instantiate sensor
        self.bme = adafruit_bme280.Adafruit_BME280_I2C(i2c, 0x76)


    def publish(self):

        # instantiate the msgs
        msg_temperature = Temperature()
        msg_humidity = RelativeHumidity()
        msg_pressure = FluidPressure()

        # insert fame id
        msg_temperature.header.frame_id = "base_link"
        msg_humidity.header.frame_id = "base_link"
        msg_pressure.header.frame_id = "base_link"

        # grab the data from the sensors
        msg_temperature.temperature = self.bme.temperature
        msg_humidity.relative_humidity = self.bme.humidity
        msg_pressure.fluid_pressure = self.bme.pressure

        # publish the data
        self.temperature_publisher.publish(msg_temperature)
        self.humidity_publisher.publish(msg_humidity)
        self.pressure_publisher.publish(msg_pressure)
