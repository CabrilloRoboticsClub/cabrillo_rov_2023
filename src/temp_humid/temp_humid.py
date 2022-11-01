'''
temp_humid.py

this ros2 node grabs temp and humid sensor data and publishes it 

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

# dependancies for ROS
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# dependancies for BME280
import board
from adafruit_bme280 import basic as adafruit_bme280

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float


class TempHumid(Node):

    def __init__(self):
        super().__init__('temp_humid')
        self.publisher_ = self.create_publisher(Float, 'temp', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # this code instanciates the sensor
        i2c = board.I2C()  # uses board.SCL and board.SDA
        bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

    def timer_callback(self):
        temp = Float()
        temp.data = bme280.temperature
        self.publisher_.publish(temp)
        self.get_logger().info('Publishing: "%s"' % temp.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()