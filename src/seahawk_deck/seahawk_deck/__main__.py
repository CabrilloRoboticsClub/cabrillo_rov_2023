'''
seahawk_deck/__main__.py

Transform joystick input to motor movement

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
import signal

import rclpy
from rclpy.node import Node

import seahawk_deck

# from .input_xbox_one import XboxInput


from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import Twist 
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Float32
from rclpy.parameter import Parameter

        """IMPORTANT: The following parameters can only use doubles as their values. Use 0.0 instead of 0 and 1.0 instead of 1."""
        self.declare_parameter('linear_x_scale', 1.0)  # Forward/Backward
        self.declare_parameter('linear_y_scale', 1.0)  # Sideways
        self.declare_parameter('linear_z_scale', 1.0)  # Depth
        self.declare_parameter('angular_x_scale', 0.0) # Roll (not using roll at the moment)
        self.declare_parameter('angular_y_scale', 0.5) # Pitch
        self.declare_parameter('angular_z_scale', 0.5) # Yaw
        self.twist_pub = self.create_publisher(Twist, 'drive/twist', 10)
        self.claw_pub = self.create_publisher(Int8MultiArray, 'solenoid', 10)
        self.cam_servo_pub = self.create_publisher(Float32, 'logic_tube/camera_servo', 10)
        self.claw_grab = False
        self.fish_release = False
        self.bambi_mode = False
        self.last_a_state = 0
        self.last_b_state = 0
        self.last_x_state = 0
        self.last_lb_state = 0
        self.last_rb_state = 0

def main(args=None):
    rclpy.init(args=args)

    try:

        node_seahawk_deck = rclpy.create_node('seahawk_deck')

        controller = seahawk_deck.XboxInput(node_seahawk_deck)


        # BINDINGS
        twist_msg = Twist()
        twist_msg.linear.x  = controller.left_stick['y'] # X (forwards)
        twist_msg.linear.y  = -controller.left_stick['x']# Y (sideways)
        twist_msg.linear.z  = (controller.left_trigger - controller.right_trigger) / 2 # Z (depth)
        twist_msg.angular.x = 0.0 # R (roll) (we don't need roll)
        twist_msg.angular.y = controller.right_stick['y'] # P (pitch) 
        twist_msg.angular.z = -controller.right_stick['x'] # Y (yaw)


        # Claw
        claw_msg = Int8MultiArray()
        claw_msg.data = [0,0,0]
        
        # Makes a button for the claw "sticky"
        if self.last_a_state == 0 and controller['a'] == 1:
            self.claw_grab = not self.claw_grab
        if self.claw_grab:
            claw_msg.data[0] = 1
        else:
            claw_msg.data[0] = 0
        self.last_a_state = controller['a']

        if self.last_b_state == 0 and controller['b'] == 1:
            self.fish_release = not self.fish_release
        if self.fish_release:
            claw_msg.data[1] = 1
        else:
            claw_msg.data[1] = 0
        self.last_b_state = controller['b']
        self.claw_pub.publish(claw_msg)

        # Makes x button for bambi mode activation "sticky" 
        if self.last_x_state == 0 and controller['x'] == 1:
            self.bambi_mode = not self.bambi_mode
        self.last_x_state = controller['x']

        cam_servo_msg = Float32()

        if controller.dpad['left']:
            cam_servo_msg.data = 0.0
        elif controller.dpad['up']:
            cam_servo_msg.data = 1.0
        elif controller.dpad['down']:
            cam_servo_msg.data = -1.0
        
        if controller.dpad['left'] or controller.dpad['up'] or controller.dpad['down']:
            self.cam_servo_pub.publish(cam_servo_msg)

        # Stores thrust values in local variables
        linear_x_scale = self.get_parameter('linear_x_scale').get_parameter_value().double_value
        linear_y_scale = self.get_parameter('linear_y_scale').get_parameter_value().double_value
        linear_z_scale = self.get_parameter('linear_z_scale').get_parameter_value().double_value
        angular_x_scale = self.get_parameter('angular_x_scale').get_parameter_value().double_value
        angular_y_scale = self.get_parameter('angular_y_scale').get_parameter_value().double_value
        angular_z_scale = self.get_parameter('angular_z_scale').get_parameter_value().double_value

        # BAMBI MODE
        # If button x is pressed, bambi mode is activated. x must be pressed again to deactivate
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
        twist_msg.angular.x *= angular_x_scale
        twist_msg.angular.y *= angular_y_scale
        twist_msg.angular.z *= angular_z_scale

        self.twist_pub.publish(twist_msg)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()



if __name__ == '__main__':
    main(sys.argv)
