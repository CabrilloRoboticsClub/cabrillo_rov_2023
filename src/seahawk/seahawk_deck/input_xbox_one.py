"""
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
"""
import sys 

import rclpy

from rclpy.node import Node 
# from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Joy
# from std_msgs.msg import Int8MultiArray
# from std_msgs.msg import Float32
# from rclpy.parameter import Parameter

ON = 1
OFF = 0

class Input(Node):
    """
    Class that implements the joystick input.
    """

    def __init__(self):
        """
        Initialize 'input_xbox_one node'
        """
        super().__init__("input_xbox_one")
        self.subscription = self.create_subscription(Joy, "joy", self.__callback, 10)
        self.twist_pub = self.create_publisher(Twist, "controller_twist", 10)
        
        self.__prev_button_state = {
            "left_stick_press" :    OFF,
            "right_stick_press" :   OFF,
            "a":                    OFF,
            "b":                    OFF,
            "x":                    OFF,
            "y":                    OFF,
            "left_bumper":          OFF,
            "right_bumper":         OFF,
            "window":               OFF,
            "menu":                 OFF,
            "xbox":                 OFF,
        }

        self.__layout = {
            "linear_y":         "left_stick_x",     # Y (sideways)
            "linear_x":         "left_stick_y",     # X (forwards)
            # unimplemented:    "left_stick_press",
            "angular_z":        "right_stick_x",    # Y (yaw)
            "angular_y":        "right_stick_y",    # P (pitch)
            # unimplemented:    "right_stick_press",
            # unimplemented:    "left_trigger",
            # unimplemented:    "right_trigger",
            # unimplemented:    "dpad_up",
            # unimplemented:    "dpad_down",
            # unimplemented:    "dpad_left",
            # unimplemented:    "dpad_right",
            # unimplemented:    "a",
            "bambi_mode":       "b",
            # unimplemented:    "x",
            # unimplemented:    "y",
            # unimplemented:    "left_bumper",
            # unimplemented:    "right_bumper",
            # unimplemented:    "window",
            # unimplemented:    "menu",
            # unimplemented:    "xbox",
        }

    def __callback(self, joy_msg: Joy):
        """
        Takes in input from the joy message from the x box and republishes it as a twist specifying 
        the direction (linear and angular x, y, z) and percent of max speed the pilot wants the robot to move

        Args:
            joy_msg: Message of type "Joy" from the joy topic
        """
        # Debug output of joy topic
        self.get_logger().debug(f"Joystick axes: {joy_msg.axes} buttons: {joy_msg.buttons}")

        # Map the values sent from the joy message to useful names
        controller = {
            # Left stick
            "left_stick_x":     -joy_msg.axes[0],
            "left_stick_y":     joy_msg.axes[1],
            "left_stick_press": joy_msg.buttons[9],
            # Right stick
            "right_stick_x":    -joy_msg.axes[3],
            "right_stick_y":    joy_msg.axes[4],
            "right_stick_press":joy_msg.buttons[10],
            # Triggers
            "left_trigger":     joy_msg.axes[2],
            "right_trigger":    joy_msg.axes[5],
            # Dpad
            "dpad_up":          int(max(joy_msg.axes[7], 0)),
            "dpad_down":        int(-min(joy_msg.axes[7], 0)),
            "dpad_left":        int(max(joy_msg.axes[6], 0)),
            "dpad_right":       int(-min(joy_msg.axes[6], 0)),
            # Buttons
            "a":                joy_msg.buttons[0],
            "b":                joy_msg.buttons[1],
            "x":                joy_msg.buttons[2],
            "y":                joy_msg.buttons[3],
            "left_bumper":      joy_msg.buttons[4],
            "right_bumper":     joy_msg.buttons[5],
            "window":           joy_msg.buttons[6],
            "menu":             joy_msg.buttons[7],
            "xbox":             joy_msg.buttons[8],
        }
        
        # ********************* Bambi mode *********************
        # Bambi mode cuts all twist values in half for more precise movements
        # Bambi mode is 'sticky', meaning a button is pressed to turn it on, and pressed again to turn it off
        bambi_div = 2 if self.__prev_button_state[self.__layout["bambi"]] == OFF and controller[self.__layout["linear_x"]] == ON else 1
        self.__prev_button_state[self.__layout["bambi"]] = controller[self.__layout["bambi"]]

        # **************** Create twist message ****************
        twist_msg = Twist()
        twist_msg.linear.x  = controller[self.__layout["linear_x"]]     / bambi_div     # Z (forwards)
        twist_msg.linear.y  = -controller[self.__layout["linear_y"]]    / bambi_div     # Y (sideways)
        # twist_msg.linear.z  = (controller["left_trigger"] - controller["right_trigger"]) / 2 # Z (depth) What????
        twist_msg.angular.x = 0.0 # R (roll) (we don"t need roll)
        twist_msg.angular.y = controller[self.__layout["angular_y"]]    / bambi_div     # P (pitch) 
        twist_msg.angular.z = -controller[self.__layout["angular_z"]]   / bambi_div     # Y (yaw)
     
        self.twist_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Input())
    rclpy.shutdown()
   

if __name__ == "__main__":
    main(sys.argv)
