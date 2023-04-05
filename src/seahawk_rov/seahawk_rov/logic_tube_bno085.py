'''
seahawk_rov/logic_tube_bno085.py

code for publishing the data from the bno085 sensor in the logic tube

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

# ros message
from sensor_msgs.msg import Imu

# inport the bno085 circuit python sensor library
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

class LogicTubeBNO085:
    def __init__(self, node, i2c):
        # instantiate the publisher
        self.publisher = node.create_publisher(Imu, 'logic_tube/imu', 8)

        # instanciate the sensor
        self.bno = BNO08X_I2C(i2c)

        # enable raw data outputs
        self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION)
        self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)

    def publish(self):
        # instantiate an imu message
        msg = Imu()

        # add the frame id
        msg.header.frame_id = "base_link"

        # load the message with data from the sensor
        # msg.linear_acceleration = self.bno.linear_acceleration
        # msg.angular_velocity = self.bno.gyro
        # msg.orientation = self.bno.geomagnetic_quaternion
        self.get_logger().info(f"Orientation  : {self.bno.geomagnetic_quaternion}")
        self.get_logger().info(f"Acceleration : {self.bno.linear_acceleration}")
        self.get_logger().info(f"Rotation     : {self.bno.gyro}")
        # publish
        self.publisher.publish(msg)
