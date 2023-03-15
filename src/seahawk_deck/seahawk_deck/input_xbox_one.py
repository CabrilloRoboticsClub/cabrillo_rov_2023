'''
input_xbox_one.py

Handle input from an Xbox One controller and output it on /drive/twist

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
from sensor_msgs.msg import Joy

class Input(Node):
    """
    Class that implements the joystick input.
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__('input_xbox_one')
        self.subscription = self.create_subscription(Joy, 'joy', self._callback, 10)
        self.twist_pub = self.create_publisher(Twist, 'drive/twist', 10)

    def _callback(self, joy_msg):
        """Called every time the joystick publishes a message."""
        self.get_logger().info(f"Joystick axes: {joy_msg.axes} buttons: {joy_msg.buttons}")

        # Compute desired motion in <x, y, z, r, p, y>

        # CONTROLLER KEYMAPPINGS
        # for more controller mappings see commit 4b3ba9b6f51c15a86bdab61bcc5f012b8b3dbd06 motion_controller.py line 41
        # https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023/blob/4b3ba9b6f51c15a86bdab61bcc5f012b8b3dbd06/src/seahawk_deck/seahawk_deck/motion_controller.py#LL41
        controller = {
            'left_stick': {
                'x':        -joy_msg.axes[0],
                'y':        joy_msg.axes[1],
                'press':    joy_msg.buttons[9],
            },
            'right_stick': {
                'x':        -joy_msg.axes[3],
                'y':        joy_msg.axes[4],
                'press':    joy_msg.buttons[10],
            },
            'left_trigger': joy_msg.axes[2],
            'right_trigger':joy_msg.axes[5],
        } 

        # BINDINGS
        twist_msg = Twist()
        twist_msg.linear.x  = controller['left_stick']['y'] # X (forwards)
        twist_msg.linear.y  = controller['left_stick']['x']# Y (sideways)
        twist_msg.linear.z  = (controller['left_trigger'] - controller['right_trigger']) / 2 # Z (depth)
        twist_msg.angular.x = 0.0 # R (roll) (we don't need roll)
        twist_msg.angular.y = controller['right_stick']['y'] # P (pitch) 
        twist_msg.angular.z = controller['right_stick']['x'] # Y (yaw)

        self.twist_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Input())
    rclpy.shutdown()    


if __name__ == '__main__':
    main(sys.argv)