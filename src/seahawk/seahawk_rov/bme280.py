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

# ros messages
from sensor_msgs.msg import FluidPressure

# import the bme280 circuit python sensor library
from adafruit_bme280 import basic as adafruit_bme280

class BME280:
    def __init__(
            self,
            node,
            i2c_bus,
            i2c_addr = 0x77,
            frame_id = "base_link",
            hardware_location = "unknown"
    ):
        self.frame_id = frame_id

        # instantiate the sensor
        self.bme = adafruit_bme280.Adafruit_BME280_I2C(i2c=i2c_bus, address=i2c_addr)

        # instantiate the publishers
        self.pressure_publisher = node.create_publisher(FluidPressure, hardware_location + '/' + 'pressure', 10)

    def publish(self):

        # instantiate the messages
        msg_pressure = FluidPressure()

        # insert frame id
        msg_pressure.header.frame_id = self.frame_id

        # get sensor data
        msg_pressure.fluid_pressure = float(self.bme.pressure)

        # publish data
        self.pressure_publisher.publish(msg_pressure)
