
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
# enable signal handling
import signal

# ros stuff
import rclpy
from rclpy.node import Node

# message types
from std_msgs.msg import Float32
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import RelativeHumidity
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

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

# import servokit for the pwm hats
from adafruit_servokit import ServoKit

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

# instanciate thrust box pwm
thrust_box_pwm = ServoKit(channels=16, i2c=i2c, address=0x41)

#### THRUSTERS PARAMS
# hard limit thrusters so they can all run at 100% no issues
# 8 motors
# 2x 30a buck
# 8/60 = 7.5
# t200 thrusters pull 7.5a at pwm 1220 in reverse and 1780 in forward
thruster_channels = (0,1,2,3,4,5,6,7)
for channel in thruster_channels:
    thrust_box_pwm.servo[channel].set_pulse_width_range(1220,1780)
    thrust_box_pwm.servo[channel].actuation_range = 3000
    thrust_box_pwm.servo[channel].angle = 1500 # zero throttle at bootup

def lerp(old_min, old_max, new_min, new_max, old_value):
    old_range = old_max - old_min
    new_range = new_max - new_min
    new_value = (((old_value - old_min) * new_range) / old_range) + new_min
    return new_value

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
    publisher_logic_tube_bme280_temperature = node_i2c_proxy.create_publisher(Temperature,'logic_tube/bme280/temperature', 8)
    publisher_logic_tube_bme280_humidity = node_i2c_proxy.create_publisher(RelativeHumidity,'logic_tube/bme280/humidity', 8)
    publisher_logic_tube_bme280_pressure = node_i2c_proxy.create_publisher(FluidPressure,'logic_tube/bme280/pressure', 8)
    publisher_thrust_box_bme280_temperature = node_i2c_proxy.create_publisher(Temperature,'thrust_box/bme280/temperature', 8)
    publisher_thrust_box_bme280_humidity = node_i2c_proxy.create_publisher(RelativeHumidity,'thrust_box/bme280/humidity', 8)
    publisher_thrust_box_bme280_pressure = node_i2c_proxy.create_publisher(FluidPressure,'thrust_box/bme280/pressure', 8)
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
        message_logic_tube_bme280_temperature.header.frame_id = "base_link"
        message_logic_tube_bme280_humidity.header.frame_id = "base_link"
        message_logic_tube_bme280_pressure.header.frame_id = "base_link"
        message_thrust_box_bme280_temperature.header.frame_id = "base_link"
        message_thrust_box_bme280_humidity.header.frame_id = "base_link"
        message_thrust_box_bme280_pressure.header.frame_id = "base_link"
#        message_logic_tube_imu.header.frame_id = "base_link"

        # grab the data from the sensors
        message_logic_tube_bme280_temperature.temperature = logic_tube_bme280.temperature
        message_logic_tube_bme280_humidity.relative_humidity = logic_tube_bme280.humidity
        message_logic_tube_bme280_pressure.fluid_pressure = logic_tube_bme280.pressure
        message_thrust_box_bme280_temperature.temperature = thrust_box_bme280.temperature
        message_thrust_box_bme280_humidity.relative_humidity = thrust_box_bme280.humidity
        message_thrust_box_bme280_pressure.fluid_pressure = thrust_box_bme280.pressure
        logic_tube_imu.enable_feature(adafruit_bno08x.BNO_REPORT_RAW_ACCELEROMETER)
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

    def thrusters_callback(msg_thrusters):
        # get the message data
        thrusters_throttle_array = msg_thrusters.data
        # set all the pwm outputs
        # im adding 32767 to the value to turn the signed int to a unsigned int
        # servo kit only works with unsigned``
        for channel in thruster_channels:
            thrust_box_pwm.servo[channel].angle = int(lerp(-1.0, 1.0, 0, 3000, thrusters_throttle_array[channel]))

    # instanciate output subscribers
    subscriber_thrusters = node_i2c_proxy.create_subscription(Float32MultiArray, 'drive/motors', thrusters_callback, 10)

    # create the timer for the i2c proxy node
    timer_i2c_proxy_publish = node_i2c_proxy.create_timer(0.1, poll_sensors)

    rclpy.spin(node_i2c_proxy)

# gracefully shutdown
def signal_handler(sig, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


if __name__ == '__main__':
    main(sys.argv)