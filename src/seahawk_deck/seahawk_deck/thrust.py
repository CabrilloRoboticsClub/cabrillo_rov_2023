'''
thrust.py

Calculate correct output of motors and output it on /drive/motors

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
import sys 

import rclpy

from rclpy.node import Node 

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class Thrust(Node):
    """
    Class that implements the kinematics.
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__('thrust')
        self.subscription = self.create_subscription(Twist, 'drive/twist', self._callback, 10)
        self.motor_pub = self.create_publisher(Float32MultiArray, 'drive/motors', 10)
        

    # Calculates what a thruster should output based on multiple input values
    def combine_input(self, direction1:float, direction2:float)->float:
        """Add two directions in such a way that they do not fall outside [-1, 1]"""
        if (direction1 >= 0 and direction2 >= 0): # If both input values are positive (0 included)
            # Combines decimal percentage values based on a probability union operation to determine what the thruster should output
            return direction1 + direction2 - (direction1 * direction2)
        elif (direction1 < 0 and direction2 < 0): # If both input values are negative
            # Tweaked probability union operation similar to above to work with negative values
            return direction1 + direction2 + (direction1 * direction2)
        else:
            # If one value is positive and one value is negative, adds the values of different signs to offset each other
            return direction1 + direction2

    def _callback(self, twist_msg):
        """Called every time the twist publishes a message."""

        # Convert the X,Y,Z,R,P,Y to thrust settings for each motor. 
        motor_msg = Float32MultiArray()
        # +1 = Full thrust, Forwards
        #  0 = Off
        # -1 = Full thrust, Backwards
        # Even are on bottom
        # 45° angle(π/4)
        # ^FRONT^
        # 6/^ ^\0
        # 4\, ,/2
        # Odd are on top; thrust forward is down
        # 35° angle(7π/36)
        # ^FRONT^
        #  7   1
        #  5   3

        motor_msg.data = [
            0.0,  # Motor 0 thrust 
            0.0,  # Motor 1 thrust
            0.0,  # Motor 2 thrust
            0.0,  # Motor 3 thrust
            0.0,  # Motor 4 thrust
            0.0,  # Motor 5 thrust
            0.0,  # Motor 6 thrust
            0.0,  # Motor 7 thrust
        ]


        # Lower motors
        motor_msg.data[0] = self.combine_input(self.combine_input(twist_msg.linear.x, twist_msg.linear.y), twist_msg.angular.z)
        motor_msg.data[2] = self.combine_input(self.combine_input(-twist_msg.linear.x, twist_msg.linear.y), -twist_msg.angular.z)
        motor_msg.data[4] = self.combine_input(self.combine_input(-twist_msg.linear.x, -twist_msg.linear.y), twist_msg.angular.z)
        motor_msg.data[6] = self.combine_input(self.combine_input(twist_msg.linear.x, -twist_msg.linear.y), -twist_msg.angular.z)

        # Upper motors
        motor_msg.data[1] = self.combine_input(-twist_msg.linear.z, twist_msg.angular.y)
        motor_msg.data[3] = self.combine_input(-twist_msg.linear.z, -twist_msg.angular.y)
        motor_msg.data[5] = self.combine_input(-twist_msg.linear.z, -twist_msg.angular.y)
        motor_msg.data[7] = self.combine_input(-twist_msg.linear.z, twist_msg.angular.y)
        

        # Publish data to the motors
        self.motor_pub.publish(motor_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Thrust())
    rclpy.shutdown()    


if __name__ == '__main__':
    main(sys.argv)
