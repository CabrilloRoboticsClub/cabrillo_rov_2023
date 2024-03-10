"""
thrust_gz_repub.py

Republishes individual motor values from `motor_encoding` to separate topics

Copyright (C) 2023-2024 Cabrillo Robotics Club

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

# For reading argv
import sys 

# ROS client library imports
import rclpy
from rclpy.node import Node 

# ROS messages imports
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray


class ThrustGzRepub(Node):
    """
    Class that republishes individual motor values from the thrust node
    """

    def __init__(self):
        """
        Initialize 'twist_gz_repub' node
        """
        super().__init__("twist_gz_repub")

        # Create publishers and subscriptions
        self.create_subscription(Float32MultiArray, "motor_values", self.callback, 10)
        
        self.publishers = []

        # One publisher for each motor
        for i in range(8):
            self.publishers.append(self.create_publisher(Float64, f"motor_values/{i}", 10))

    def callback(self, msg: Float32MultiArray):
        """
        Republishes individual motor values from the thrust node on unique topics

        Args:
            msg: Message from `motor_encoding` topic where each element of the array corresponds 
                 to the newtons of force generated by each motor.
        """
        for motor_val, pub in zip(msg.data, self.publishers):
            pub.publish(motor_val)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ThrustGzRepub())
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)