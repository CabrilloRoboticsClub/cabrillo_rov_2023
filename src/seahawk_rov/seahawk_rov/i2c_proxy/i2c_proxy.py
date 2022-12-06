
'''
i2c_proxy.py

this is the node with the lock on /dev/i2c
this node publishes the data from the i2c sensors in the robot
this node subscribes to the signals for the i2c signal generators on the robot

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

# enable command line arguments
import sys

# ros stuff
import rclpy
from rclpy.node import Node

# message types
from std_msgs.msg import Float32

# this library is needed by the bno085
import time

# library for accessing the raspberry pi board
# aka /dev/i2c
import board
import busio

# import the bme280 circuit python sensor library
from adafruit_bme280 import basic as adafruit_bme280

# inport the bno085 circuit python sensor library
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C


# # # # # # # #
#
# INSTANCIATIONS
#
# # # # # # # #

# grab the i2c interface for us to use
i2c = board.I2C

# instanciate the logic tube bme280
# enviromental sensor
logic_tube_bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, 0x77)

# instanciate the thrust box bme280
# enviromental sensor
thrust_box_bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, 0x76)

# instanciate the logic tube bmo085
# 9dof absolute orientation imu Ssensor
logic_tube_imu = BNO08X_I2C(i2c)


# # # # # # # #
#
# main
#
# # # # # # # #

def main(args=None):
    rclpy.init(args=args)

    # this creates the node "i2c_proxy"
    node_i2c_proxy = rclpy.create_node('i2c_proxy')

    # this creates a publisher for logic tube bme280 temp of message type Float32 with name "logic_tube_bme280_tempurature" with a history of 8 messages
    publisher_logic_tube_bme280_tempurature = node_i2c_proxy.create_publisher(Float32,'logic_tube_bme280_tempurature', 8)

    publisher_logic_tube_bme280_humidity = node_i2c_proxy.create_publisher(Float32,'logic_tube_bme280_humidity', 8)

    publisher_logic_tube_bme280_pressure = node_i2c_proxy.create_publisher(Float32,'logic_tube_bme280_pressure', 8)

    publisher_thrust_box_bme280_tempurature = node_i2c_proxy.create_publisher(Float32,'thrust_box_bme280_tempurature', 8)

    publisher_thrust_box_bme280_humidity = node_i2c_proxy.create_publisher(Float32,'thrust_box_bme280_humidity', 8)

    publisher_thrust_box_bme280_pressure = node_i2c_proxy.create_publisher(Float32,'thrust_box_bme280_pressure', 8)

    def poll_sensors():
        msg = Float32()

        msg.data = logic_tube_bme280.temperature
        publisher_logic_tube_bme280_tempurature.publish(msg)

        msg.data = logic_tube_bme280.humidity
        publisher_logic_tube_bme280_humidity.publish(msg)

        msg.data = logic_tube_bme280.pressure
        publisher_logic_tube_bme280_pressure.publish(msg)

        msg.data = thrust_box_bme280.tempurature
        publisher_thrust_box_bme280_tempurature.publish(msg)

    # create the timer for the i2c proxy node
    timer = node_i2c_proxy.create_timer(0.1, poll_sensors)

    rclpy.spin(node_i2c_proxy)

# # # # # # # #
#
# SENSOR DATA GRAB CODE
#
# # # # # # # #

# logic tube enviroment
print("logic tube Temperature: %0.1f C" % logic_tube_bme280.temperature)
print("logic tube Humidity: %0.1f %%" % logic_tube_bme280.humidity)
print("logic tube Pressure: %0.1f hPa" % logic_tube_bme280.pressure)

# thust box enviroment
print("thrust box Temperature: %0.1f C" % thrust_box_bme280.temperature)
print("thrust box Humidity: %0.1f %%" % thrust_box_bme280.humidity)
print("thrust box Pressure: %0.1f hPa" % thrust_box_bme280.pressure)

# logic tube imu
print("\n logic tube imu")
# insert snesor reading code here





if __name__ == '__main__':
    main(sys.argv)