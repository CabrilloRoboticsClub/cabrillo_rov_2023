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
from rcl_interfaces.srv import SetParameters
# from rcl_interfaces.msg import Parameter, ParameterValue
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Joy
from rclpy.parameter import Parameter

class Input(Node):
    """
    Class that implements the joystick input.
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__('input_xbox_one')
        """IMPORTANT: The following parameters can only use doubles as their values. Use 0.0 instead of 0 and 1.0 instead of 1."""
        self.declare_parameter('linear_x_scale', 1.0) # Forward/Backward
        self.declare_parameter('linear_y_scale', 1.0) # Sideways
        self.declare_parameter('linear_z_scale', 1.0) # Depth
        self.declare_parameter('angular_x_scale', 0.0) # Roll (not using roll at the moment)
        self.declare_parameter('angular_y_scale', 0.5) # Pitch
        self.declare_parameter('angular_z_scale', 0.5) # Yaw
        self.subscription = self.create_subscription(Joy, 'joy', self._callback, 10)
        self.twist_pub = self.create_publisher(Twist, 'drive/twist', 10)
        self.bambi_mode = False
        self.last_x_state = 0
    

    def _callback(self, joy_msg):
        """Called every time the joystick publishes a message."""
        self.get_logger().info(f"Joystick axes: {joy_msg.axes} buttons: {joy_msg.buttons}")
        # self._bambi = self.get_parameter_or('/thrust/angular_x_scale', 'FUCK')
        # self.get_logger().info(self._bambi)

        # Compute desired motion in <x, y, z, r, p, y>

        # CONTROLLER KEYMAPPINGS
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
            'dpad': {
                'up':       max(joy_msg.axes[7], 0), # +
                'down':     min(joy_msg.axes[7], 0), # -
                'right':    max(joy_msg.axes[6], 0), # +
                'left':     min(joy_msg.axes[6], 0), # -
            },
            'a':            joy_msg.buttons[0],
            'b':            joy_msg.buttons[1],
            'x':            joy_msg.buttons[2], # bambi (scale everything by half to reduce speed)
            'y':            joy_msg.buttons[3],
            'left_bumper':  joy_msg.buttons[4],
            'right_bumper': joy_msg.buttons[5],
            'window':       joy_msg.buttons[6],
            'menu':         joy_msg.buttons[7],
            'xbox':         joy_msg.buttons[8],
        } 

        # BINDINGS
        twist_msg = Twist()
        twist_msg.linear.x  = controller['left_stick']['y'] # X (forwards)
        twist_msg.linear.y  = controller['left_stick']['x']# Y (sideways)
        twist_msg.linear.z  = (controller['left_trigger'] - controller['right_trigger']) / 2 # Z (depth)
        twist_msg.angular.x = 0.0 # R (roll) (we don't need roll)
        twist_msg.angular.y = controller['right_stick']['y'] # P (pitch) 
        twist_msg.angular.z = controller['right_stick']['x'] # Y (yaw)
        
        # Makes x button for bambi mode activation "sticky" 
        if self.last_x_state == 0 and controller['x'] == 1:
            self.bambi_mode = not self.bambi_mode
        self.last_x_state = controller['x']

        # Stores thrust values in local variables
        linear_x_scale = self.get_parameter('linear_x_scale').get_parameter_value().double_value
        linear_y_scale = self.get_parameter('linear_y_scale').get_parameter_value().double_value
        linear_z_scale = self.get_parameter('linear_z_scale').get_parameter_value().double_value
        angular_x_scale = self.get_parameter('angular_x_scale').get_parameter_value().double_value
        angular_y_scale = self.get_parameter('angular_y_scale').get_parameter_value().double_value
        angular_z_scale = self.get_parameter('angular_z_scale').get_parameter_value().double_value

        # BAMBI MODE
        # If button x is pressed, bambi node is activated. x must be pressed again to deactivate
        # Bambi mode cuts all motor thrust in half
        if self.bambi_mode:
            linear_x_scale /= 2
            linear_y_scale /= 2
            linear_z_scale /= 2
            angular_x_scale /= 2
            angular_y_scale /= 2
            angular_z_scale /= 2

        # AXIS SCALE
        twist_msg.linear.x  *= linear_x_scale
        twist_msg.linear.y  *= linear_y_scale
        twist_msg.linear.z  *= linear_z_scale
        twist_msg.angular.x *= angular_x_scale
        twist_msg.angular.y *= angular_y_scale
        twist_msg.angular.z *= angular_z_scale

        self.twist_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Input())
    rclpy.shutdown()    


if __name__ == '__main__':
    main(sys.argv)