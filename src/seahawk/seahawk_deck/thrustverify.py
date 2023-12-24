"""
thrustverify.py

Calculate predicted thrust and angular vector based on motor status and output it on /drive/predict

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
"""
import sys

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class ThrustVerify(Node):
    """Caculates expected input, based on current motor status"""

    def __init__(self):
        super().__init__("thrust")

        self.subscription = self.create_subscription(
            Float32MultiArray, "drive/motors", self.motor_math, 10
        )
        self.vector_predict = self.create_publisher(
            Twist, "drive/predict", 10
        )
        self.motor_config = [
            [     0,     0,    0,     0,     0.7071,     0.7071,    -0.7071,   -0.7071 ],  # Fx (N)
            [     0,     0,    0,     0,    -0.7071,     0.7071,    -0.7071,    0.7071 ],  # Fy (N)
            [     1,     1,    1,     1,          0,          0,          0,         0 ],  # Fz (N)
            [  0.12, -0.12, 0.12, -0.12, -0.0268698,  0.0268698, -0.0268698, 0.0268698 ],  # Rx (N*m)
            [ -0.19, -0.19, 0.19,  0.19, -0.0268698, -0.0268698,  0.0268698, 0.0268698 ],  # Ry (N*m)
            [     0,     0,    0,     0,  -0.180311,   0.180311,   0.180311, -0.180311 ],  # Rz (N*m)
        ]

    def motor_math(self, msg):
        # motor 0-7 within list index
        motor_msg = msg

        # Linear Thrust Vector -- Operable
        x = (
              self.motor_config[0][4] * motor_msg.data[4]
            + self.motor_config[0][5] * motor_msg.data[5]
            + self.motor_config[0][6] * motor_msg.data[6]
            + self.motor_config[0][7] * motor_msg.data[7]
        )
        y = (
              self.motor_config[1][4] * motor_msg.data[4]
            + self.motor_config[1][5] * motor_msg.data[5]
            + self.motor_config[1][6] * motor_msg.data[6]
            + self.motor_config[1][7] * motor_msg.data[7]
        )
        z = (
              self.motor_config[2][0] * motor_msg.data[0]
            + self.motor_config[2][1] * motor_msg.data[1]
            + self.motor_config[2][2] * motor_msg.data[2]
            + self.motor_config[2][3] * motor_msg.data[3]
        )

        # Angular Thrust Vector -- Inoperable
        rx = (
              self.motor_config[3][0] * motor_msg.data[0]
            + self.motor_config[3][1] * motor_msg.data[1]
            + self.motor_config[3][2] * motor_msg.data[2]
            + self.motor_config[3][3] * motor_msg.data[3]
            + self.motor_config[3][4] * motor_msg.data[4]
            + self.motor_config[3][5] * motor_msg.data[5]
            + self.motor_config[3][6] * motor_msg.data[6]
            + self.motor_config[3][7] * motor_msg.data[7]
        )
        ry = (
              self.motor_config[4][0] * motor_msg.data[0]
            + self.motor_config[4][1] * motor_msg.data[1]
            + self.motor_config[4][2] * motor_msg.data[2]
            + self.motor_config[4][3] * motor_msg.data[3]
            + self.motor_config[4][4] * motor_msg.data[4]
            + self.motor_config[4][5] * motor_msg.data[5]
            + self.motor_config[4][6] * motor_msg.data[6]
            + self.motor_config[4][7] * motor_msg.data[7]
        )
        rz = (
              self.motor_config[5][0] * motor_msg.data[0]
            + self.motor_config[5][1] * motor_msg.data[1]
            + self.motor_config[5][2] * motor_msg.data[2]
            + self.motor_config[5][3] * motor_msg.data[3]
            + self.motor_config[5][4] * motor_msg.data[4]
            + self.motor_config[5][5] * motor_msg.data[5]
            + self.motor_config[5][6] * motor_msg.data[6]
            + self.motor_config[5][7] * motor_msg.data[7]
        )

        predicted_vectors = Twist()

        predicted_vectors.linear.x = x
        predicted_vectors.linear.y = y
        predicted_vectors.linear.z = z
        predicted_vectors.angular.x = rx
        predicted_vectors.angular.y = ry
        predicted_vectors.angular.z = rz

        self.vector_predict.publish(predicted_vectors)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ThrustVerify())
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
