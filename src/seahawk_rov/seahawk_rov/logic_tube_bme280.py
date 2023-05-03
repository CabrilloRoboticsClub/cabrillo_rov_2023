'''
seahawk_rov/logic_tube_bme280.py

code for publishing the data from the bme280 sensor in the logic tube

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

class Config:
    def __init__(self, node, hardware_location = "unknown_hardware_location/", i2c_bus, i2c_addr = 0x77):

        self.node = node # the ROS2 Node to which the publishers will be attached.
        #TODO: Ensure hardware_location has a trailing slash. It's important to our topic naming pattern. ("container/sensorvalue" e.g. "logic_tube/temperature")
        self.hardware_location = hardware_location # Prefix for the topic names. This one should be "logic_tube", but it's defaulted to a generic "unknown..." to better indicate bad usage by the caller.
        self.i2c_bus = i2c_bus # I2C object. It'll be whatever is returned by busio.I2C(), probably.
        self.i2c_addr = i2c_addr # The device's I2C address. Defaulted to 0x77.

        # I'm not sure how to record the "t", "h", and "p". In the config. I don't know if it makes sense, though.
        # The sensor, by virtue of being *this* sensor, simply has those properties. Being a LogicTubeBME280 means
        # that there is a t, h, and p. It doesn't make sense to ask it to have 2 pressures, for instance.

class LogicTubeBME280:
    def __init__(self, config):
        
        # the configuration is passed in. Immediately assigning a new value isn't what we want.

        # instanciate the publishers
        # With a `class Config` we can call the property names by... well by name.
        self.temperature_publisher = config.node.create_publisher(Temperature, config.hardware_location + 'temperature', 10)
        self.humidity_publisher = config.node.create_publisher(RelativeHumidity,config.hardware_location + 'humidity', 10)
        self.pressure_publisher = config.node.create_publisher(FluidPressure, config.hardware_location + 'pressure', 10)

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

    def poll(self):

        # get sensor data
        self.msg_temperature.temperature = self.bme.temperature
        self.msg_humidity.relative_humidity = self.bme.humidity
        self.msg_pressure.fluid_pressure = self.bme.pressure

    def publish(self):

        # publish data
        self.temperature_publisher.publish(self.msg_temperature)
        self.humidity_publisher.publish(self.msg_humidity)
        self.pressure_publisher.publish(self.msg_pressure)
