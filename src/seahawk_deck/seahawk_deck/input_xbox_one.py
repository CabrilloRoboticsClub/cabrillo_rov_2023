'''
input_xbox_one.py

Handle input from an Xbox One controller and abstract it

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
from sensor_msgs.msg import Joy

class XboxInput(Node):
    """
    Class that implements the joystick input.
    """

    def __init__(self, node):
        """Initialize this node"""
        super().__init__('input_xbox_one')
        self.joy_sub = node.create_subscription(Joy, 'joy', self._callback, 10)
    

    def _callback(self, joy_msg:Joy):
        """Called every time the joystick publishes a message."""
        self.get_logger().debug(f"Joystick axes: {joy_msg.axes} buttons: {joy_msg.buttons}")

        # Compute desired motion in <x, y, z, r, p, y>

        # CONTROLLER KEYMAPPINGS
        self.left_stick = {
            'x':        -joy_msg.axes[0],
            'y':        joy_msg.axes[1],
            'press':    joy_msg.buttons[9],
        }
        self.right_stick = {
            'x':        -joy_msg.axes[3],
            'y':        joy_msg.axes[4],
            'press':    joy_msg.buttons[10],
        }
        self.left_trigger  = joy_msg.axes[2]
        self.right_trigger = joy_msg.axes[5]
        self.dpad = {
            'up':       int(max(joy_msg.axes[7], 0)), # +
            'down':     int(-min(joy_msg.axes[7], 0)), # -
            'left':     int(max(joy_msg.axes[6], 0)), # +
            'right':    int(-min(joy_msg.axes[6], 0)), # -
        }
        self.buttons = {
            'a':            joy_msg.buttons[0],
            'b':            joy_msg.buttons[1],
            'x':            joy_msg.buttons[2], # bambi (scale everything by half to reduce speed)
            'y':            joy_msg.buttons[3],
            'left_bumper':  joy_msg.buttons[4], # trim -
            'right_bumper': joy_msg.buttons[5], # trim +
            'window':       joy_msg.buttons[6],
            'menu':         joy_msg.buttons[7],
            'xbox':         joy_msg.buttons[8],
        }