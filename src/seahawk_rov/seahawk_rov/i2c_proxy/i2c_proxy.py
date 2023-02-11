
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
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import RelativeHumidity
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Imu

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
i2c = board.I2C()

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

    # instanciate sensor publishers
    publisher_logic_tube_bme280_temperature = node_i2c_proxy.create_publisher(Temperature,'logic_tube_bme280_temperature', 8)
    publisher_logic_tube_bme280_humidity = node_i2c_proxy.create_publisher(RelativeHumidity,'logic_tube_bme280_humidity', 8)
    publisher_logic_tube_bme280_pressure = node_i2c_proxy.create_publisher(FluidPressure,'logic_tube_bme280_pressure', 8)
    publisher_thrust_box_bme280_temperature = node_i2c_proxy.create_publisher(Temperature,'thrust_box_bme280_temperature', 8)
    publisher_thrust_box_bme280_humidity = node_i2c_proxy.create_publisher(RelativeHumidity,'thrust_box_bme280_humidity', 8)
    publisher_thrust_box_bme280_pressure = node_i2c_proxy.create_publisher(FluidPressure,'thrust_box_bme280_pressure', 8)
#    publisher_logic_tube_imu = node_i2c_proxy.create_publisher(Imu, 'logic_tube_imu', 8)

    def poll_sensors():

        # instanciate the messages
        message_logic_tube_bme280_temperature = Temperature()
        message_logic_tube_bme280_humidity = RelativeHumidity()
        message_logic_tube_bme280_pressure = FluidPressure()
        message_thrust_box_bme280_temperature = Temperature()
        message_thrust_box_bme280_humidity = RelativeHumidity()
        message_thrust_box_bme280_pressure = FluidPressure()
#        message_logic_tube_imu = Imu()

        # insert fame id
        message_logic_tube_bme280_temperature.header.frame-id = "base_link"
        message_logic_tube_bme280_humidity.header.frame-id = "base_link"
        message_logic_tube_bme280_pressure.header.frame-id = "base_link"
        message_thrust_box_bme280_temperature.header.frame-id = "base_link"
        message_thrust_box_bme280_humidity.header.frame-id = "base_link"
        message_thrust_box_bme280_pressure.header.frame-id = "base_link"

        # grab the data from the sensors
        message_logic_tube_bme280_temperature.temperature = logic_tube_bme280.temperature
        message_logic_tube_bme280_humidity.relative_humidity = logic_tube_bme280.humidity
        message_logic_tube_bme280_pressure.fluid_pressure = logic_tube_bme280.pressure
        message_thrust_box_bme280_temperature.temperature = thrust_box_bme280.temperature
        message_thrust_box_bme280_humidity.relative_humidity = thrust_box_bme280.humidity
        message_thrust_box_bme280_pressure.fluid_pressure = thrust_box_bme280.pressure
#        logic_tube_imu.enable_feature(adafruit_bno08x.BNO_REPORT_RAW_ACCELEROMETER)
#        message_logic_tube_imu.linear_acceleration = logic_tube_imu.raw_acceleration
#        logic_tube_imu.enable_feature(adafruit_bno08x.BNO_REPORT_RAW_GYROSCOPE)
#        message_logic_tube_imu.angular_velocity = logic_tube_imu.raw_gyro
#        logic_tube_imu.enable_feature(adafruit_bno08x.BNO_REPORT_RAW_MAGNETOMETER)
#        message_logic_tube_imu.orientation = logic_tube_imu.raw_quaternion

        # pubblish the data
        publisher_logic_tube_bme280_temperature.publish(message_logic_tube_bme280_temperature)
        publisher_logic_tube_bme280_humidity.publish(message_logic_tube_bme280_humidity)
        publisher_logic_tube_bme280_pressure.publish(message_logic_tube_bme280_pressure)
        publisher_thrust_box_bme280_temperature.publish(message_thrust_box_bme280_temperature)
        publisher_thrust_box_bme280_humidity.publish(message_thrust_box_bme280_humidity)
        publisher_thrust_box_bme280_pressure.publish(message_thrust_box_bme280_pressure)
#        publisher_logic_tube_imu.publish(message_logic_tube_imu)
        
    # create the timer for the i2c proxy node
    timer_i2c_proxy_publish = node_i2c_proxy.create_timer(0.1, poll_sensors)

    rclpy.spin(node_i2c_proxy)



if __name__ == '__main__':
    main(sys.argv)