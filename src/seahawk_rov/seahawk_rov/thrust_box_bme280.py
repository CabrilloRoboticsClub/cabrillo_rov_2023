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
        self.temperature_publisher = node.create_publisher(Temperature,'thrust_box/temperature', 10)
        self.humidity_publisher = node.create_publisher(RelativeHumidity,'thrust_box/humidity', 10)
        self.pressure_publisher = node.create_publisher(FluidPressure,'thrust_box/pressure', 10)

        # instantiate sensor
        self.bme = adafruit_bme280.Adafruit_BME280_I2C(i2c, 0x76)

    # instanciate the messages
        self.msg_temperature = Temperature()
        self.msg_humidity = RelativeHumidity()
        self.msg_pressure = FluidPressure()

        # insert frame id
        self.msg_temperature.header.frame_id = "base_link"
        self.msg_humidity.header.frame_id = "base_link"
        self.msg_pressure.header.frame_id = "base_link"

        # instanciate the sensor
        self.bme = adafruit_bme280.Adafruit_BME280_I2C(i2c, 0x77)

    def poll_sensors(self):

        # get sensor data
        self.msg_temperature.temperature = self.bme.temperature
        self.msg_humidity.relative_humidity = self.bme.humidity
        self.msg_pressure.fluid_pressure = self.bme.pressure

    def publish(self):

        # publish data
        self.temperature_publisher.publish(self.msg_temperature)
        self.humidity_publisher.publish(self.msg_humidity)
        self.pressure_publisher.publish(self.msg_pressure)
