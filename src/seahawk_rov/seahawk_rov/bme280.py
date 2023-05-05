'''
seahawk_rov/bme280.py

code for publishing data from bme280 sensors

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

# ros messages
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import RelativeHumidity
from sensor_msgs.msg import FluidPressure

# import the bme280 circuit python sensor library
from adafruit_bme280 import basic as adafruit_bme280

class BME280:
    def __init__(
            self,
            node,
            callback_group,
            i2c_bus,
            i2c_addr = 0x77,
            hardware_location = "unknown_hardware_location/"
    ):
        # the ros2 node the publishers will be attached to
        self.node = node

        # which callback group in threading does this code beling to
        self.callback_group = callback_group

        # topic name prefix for the hardware location
        # in seahawk that would either be "logic_tube" or "thrust_box"
        self.hardware_location = hardware_location

        # the i2c bus object from board.i2c in the main file
        self.i2c_bus = i2c_bus

        # the address the sensor is at on the i2c bus
        # for the adafruit bme280 the unsoldered addr is 0x77 and soldered is 0x76
        self.i2c_addr = i2c_addr

        # instanciate the publishers
        self.temperature_publisher = config.node.create_publisher(Temperature, config.hardware_location + '/temperature', 10)
        self.humidity_publisher = config.node.create_publisher(RelativeHumidity,config.hardware_location + '/humidity', 10)
        self.pressure_publisher = config.node.create_publisher(FluidPressure, config.hardware_location + '/pressure', 10)

        # instanciate the messages
        self.msg_temperature = Temperature()
        self.msg_humidity = RelativeHumidity()
        self.msg_pressure = FluidPressure()

        # insert frame id
        self.msg_temperature.header.frame_id = "base_link"
        self.msg_humidity.header.frame_id = "base_link"
        self.msg_pressure.header.frame_id = "base_link"

        # instanciate the sensor
        self.bme = adafruit_bme280.Adafruit_BME280_I2C(config.i2c_bus, config.i2c_addr)

        # create the timer
        publish_timer = config.node.create_timer(1, self.publish, callback_group=config.callback_group)

    def publish(self):

        # get sensor data
        self.msg_temperature.temperature = self.bme.temperature
        self.msg_humidity.relative_humidity = self.bme.humidity
        self.msg_pressure.fluid_pressure = self.bme.pressure

        # publish data
        self.temperature_publisher.publish(self.msg_temperature)
        self.humidity_publisher.publish(self.msg_humidity)
        self.pressure_publisher.publish(self.msg_pressure)
