
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
# sensor publisher class
#
# # # # # # # #

class SensorPublisher:
    def __init__(self, node, i2c):
        # instanciate sensor publishers
        self.logic_tube_temperature = node.create_publisher(Temperature,'logic_tube/temperature', 8)
        self.logic_tube_humidity = node.create_publisher(RelativeHumidity,'logic_tube/humidity', 8)
        self.logic_tube_pressure = node.create_publisher(FluidPressure,'logic_tube/pressure', 8)
        self.thrust_box_temperature = node.create_publisher(Temperature,'thrust_box/temperature', 8)
        self.thrust_box_humidity = node.create_publisher(RelativeHumidity,'thrust_box/humidity', 8)
        self.thrust_box_pressure = node.create_publisher(FluidPressure,'thrust_box/pressure', 8)
        self.logic_tube_imu = node.create_publisher(Imu, 'logic_tube/imu', 8)

        # instanciate sensors
        self.logic_tube_bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, 0x77)
        self.thrust_box_bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, 0x76)
        self.logic_tube_imu = BNO08X_I2C(i2c)

        # configure sensors
        self.logic_tube_imu.enable_feature(adafruit_bno08x.BNO_REPORT_RAW_ACCELEROMETER)
        self.logic_tube_imu.enable_feature(adafruit_bno08x.BNO_REPORT_RAW_GYROSCOPE)
        self.logic_tube_imu.enable_feature(adafruit_bno08x.BNO_REPORT_RAW_MAGNETOMETER)

        # instanciate timer
        self.timer = node.create_timer(0.1, self.publish)

 
    def publish(self):

        # instanciate the messages
        message_logic_tube_temperature = Temperature()
        message_logic_tube_humidity = RelativeHumidity()
        message_logic_tube_pressure = FluidPressure()
        message_thrust_box_temperature = Temperature()
        message_thrust_box_humidity = RelativeHumidity()
        message_thrust_box_pressure = FluidPressure()
        message_logic_tube_imu = Imu()

        # insert fame id
        message_logic_tube_temperature.header.frame_id = "base_link"
        message_logic_tube_humidity.header.frame_id = "base_link"
        message_logic_tube_pressure.header.frame_id = "base_link"
        message_thrust_box_temperature.header.frame_id = "base_link"
        message_thrust_box_humidity.header.frame_id = "base_link"
        message_thrust_box_pressure.header.frame_id = "base_link"
        message_logic_tube_imu.header.frame_id = "base_link"

        # grab the data from the sensors
        message_logic_tube_temperature.temperature = self.logic_tube_bme280.temperature
        message_logic_tube_humidity.relative_humidity = self.logic_tube_bme280.humidity
        message_logic_tube_pressure.fluid_pressure = self.logic_tube_bme280.pressure
        message_thrust_box_temperature.temperature = self.thrust_box_bme280.temperature
        message_thrust_box_humidity.relative_humidity = self.thrust_box_bme280.humidity
        message_thrust_box_pressure.fluid_pressure = self.thrust_box_bme280.pressure

        #message_logic_tube_imu.linear_acceleration = self.logic_tube_imu.raw_acceleration
        #message_logic_tube_imu.angular_velocity = self.logic_tube_imu.raw_gyro
        #message_logic_tube_imu.orientation = self.logic_tube_imu.raw_quaternion

        # publish the data
        self.logic_tube_temperature.publish(message_logic_tube_temperature)
        self.logic_tube_humidity.publish(message_logic_tube_humidity)
        self.logic_tube_pressure.publish(message_logic_tube_pressure)
        self.thrust_box_temperature.publish(message_thrust_box_temperature)
        self.thrust_box_humidity.publish(message_thrust_box_humidity)
        self.thrust_box_pressure.publish(message_thrust_box_pressure)
        #self.logic_tube_imu.publish(message_logic_tube_imu)



# # # # # # # # #
#
# output subscriber class
#
# # # # # # # # #

class OutputSubscriber:
    def __init__(self, node, i2c):
        # create subscribers
        self.thrusters = node.create_subscription(Float32MultiArray, 'drive/motors', self.receive_thruster, 10)
        self.drive_cam = node.create_subscription(Float32, 'camera_control', self.receive_drive_camera, 10)

        # create member variables
        self.drive_cam_servo = 15
        self.thrusters = (0,1,2,3,4,5,6,7)

        # instanciate outputs
        self.logic_tube_pwm = ServoKit(channels=16, i2c=i2c, address=0x40)
        self.thrust_box_pwm = ServoKit(channels=16, i2c=i2c, address=0x41)

        # configure outputs
        for thruster in self.thrusters:
            self.thrust_box_pwm.servo[thruster].set_pulse_width_range(1220, 1780)
            self.thrust_box_pwm.servo[thruster].actuation_range = 3000
            self.thrust_box_pwm.servo[thruster].angle = 1500 # zero throttle at bootup
        self.logic_tube_pwm.servo[self.drive_cam_servo].set_pulse_width_range(0, 3000)
        self.logic_tube_pwm.servo[self.drive_cam_servo].actuation_range = 3000
        self.logic_tube_pwm.servo[self.drive_cam_servo].angle = 1500


    def receive_thruster(self, message:Float32MultiArray):
        for thruster in self.thrusters:
            self.thrust_box_pwm.servo[thruster].angle = int(lerp(-1.0, 1.0, 0, 3000, clamp(message.data[thruster], -1, 1)))

    def receive_drive_camera(self, message:Float32):
        self.logic_tube_pwm.servo[self.drive_cam_servo].angle = int(lerp(-1.0, 1.0, 0, 3000, clamp(message.data, -1, 1)))


# # # # # # # #
#
# Helper Functions
#
# # # # # # # #

def lerp(old_min:float, old_max:float, new_min:int, new_max:int, old_value:float):
    old_range = old_max - old_min
    new_range = new_max - new_min
    new_value = (((old_value - old_min) * new_range) / old_range) + new_min
    return new_value

def clamp(num, minimum, maximum):
  return max(min(minimum, num), maximum)

# # # # # # # #
#
# main
#
# # # # # # # #

def main(args=None):
    rclpy.init(args=args)

    # this creates the node "i2c_proxy"
    node_i2c_proxy = rclpy.create_node('i2c_proxy')

    # grab the i2c interface for us to use
    i2c = board.I2C()

    sensor_publisher = SensorPublisher(node_i2c_proxy, i2c)

    output_subscriber = OutputSubscriber(node_i2c_proxy, i2c)

    rclpy.spin(node_i2c_proxy)

# # # # # # # #
#
# gracefull shutdown
#
# # # # # # # #
def signal_handler(sig, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


if __name__ == '__main__':
    main(sys.argv)