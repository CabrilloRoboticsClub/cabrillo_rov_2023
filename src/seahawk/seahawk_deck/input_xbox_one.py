"""
input_xbox_one.py

Handle input from an Xbox One controller

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
from std_msgs.msg import Bool

from scipy.interpolate import CubicSpline
# from std_msgs.msg import Int8MultiArray
# from std_msgs.msg import Float32
# from rclpy.parameter import Parameter


ON = 1
OFF = 0


class StickyButton():
    """
    Class that implements sticky buttons, meaning a button is pressed to turn it on, 
    and pressed again to turn it off
    """

    def __init__(self):
        """
        Initialize 'StickyButton' object
        """
        self.__feature_state = OFF
        self.__track_state = 0b0000
    
    def check_state(self, cur_button_state: bool) -> bool:
        """
        Checks if a button is toggled on or off and accounts for debouncing

        Args:
            cur_button_state: Current state of the button provided by the controller

        Returns:
            True if the button is toggled on, False if off
        """
        self.__track_state = (self.__track_state << 1 | cur_button_state) & 0x0F

        if self.__track_state == 0b0011:
            self.__feature_state = not self.__feature_state
        return bool(self.__feature_state)

class InputXboxOne(Node):
    """
    Class that implements the joystick input
    """

    def __init__(self):
        """
        Initialize 'input_xbox_one' node
        """
        super().__init__("input_xbox_one")

        self.subscription = self.create_subscription(Joy, "joy", self.__callback, 10)
        self.__twist_pub = self.create_publisher(Twist, "controller_twist", 10)
        self.__claw_pub = self.create_publisher(Bool, "claw", 10)
        
        self.__buttons = {
            # "" :                StickyButton(),        # left_stick_press
            # "" :                StickyButton(),        # right_stick_press
            "claw":             StickyButton(),       # a
            "bambi_mode":       StickyButton(),       # b
            # "":                 StickyButton(),        # x
            # "":                 StickyButton(),        # y
            # "":                 StickyButton(),        # left_bumper
            # "":                 StickyButton(),        # right_bumper
            # "":                 StickyButton(),        # window
            # "":                 StickyButton(),        # menu
            # "":                 StickyButton(),        # xbox
        }
    
        # self.__cur_throttle_curve = 0
        # self.__throttle_y = []
        # self.__throttle_x = [0.0, 0.25, 0.50, 0.75, 1.0]
    
    def __throttle_curve(self, input: float, curve: int=0):
        """
        Applies a throttle curve to the 'input'. A throttle curve allows the user to
        modify the relationship between the stick position and the actual throttle value sent
        to the motors

        The throttle curve is selected by passing a number [0, 2] to the 'curve' param
            0 (default): No throttle curve
            1: [0.0, 0.4, 0.7, 0.8, 1.0]
            2: [0.0, 0.1, 0.2, 0.5, 1.0]
    
        Args:
            input: The value to remap
            curve: The type of curve to remap to

        Returns:
            'input' remapped to fit the specified throttle curve
        """
        # if curve == self.__cur_throttle_curve:
        #     pass
        # else:
        match curve:
            case 1:
                y = [0.0, 0.4, 0.7, 0.8, 1.0]
            case 2:
                y = [0.0, 0.1, 0.2, 0.5, 1.0]
            case _:
                return input

        x = [0.0, 0.25, 0.50, 0.75, 1.0]

        # Determine if we need to multiply by a negative to indicate the direction
        direction = -1 if input < 0 else 1

        # CubicSpline creates a cubic spline interpolation given a list of x and y values
        # It returns an interpolated function
        return direction * CubicSpline(x, y)(abs(input))

    def __callback(self, joy_msg: Joy):
        """
        Takes in input from the joy message from the x box and republishes it as a twist specifying 
        the direction (linear and angular x, y, z) and percent of max speed the pilot wants the robot to move

        Args:
            joy_msg: Message of type 'Joy' from the joy topic
        """

        # Debug output of joy topic
        self.get_logger().debug(f"Joystick axes: {joy_msg.axes} buttons: {joy_msg.buttons}")

        # Map the values sent from the joy message to useful names
        controller = {
            # Left stick
            "linear_y":         joy_msg.axes[0],                # left_stick_x
            "linear_x":         joy_msg.axes[1],                # left_stick_y
            # "":                 joy_msg.buttons[9],             # left_stick_press
            # Right stick
            "angular_z":        joy_msg.axes[3],                # right_stick_x
            "angular_y":        joy_msg.axes[4],                # right_stick_y
            # "":                 joy_msg.buttons[10],            # right_stick_press
            # Triggers
            "neg_linear_z":     joy_msg.axes[2],                # left_trigger
            "pos_linear_z":     joy_msg.axes[5],                # right_trigger
            # Dpad
            # "":                 int(max(joy_msg.axes[7], 0)),   # dpad_up
            # "":                 int(-min(joy_msg.axes[7], 0)),  # dpad_down
            # "":                 int(max(joy_msg.axes[6], 0)),   # dpad_left     
            # "":                 int(-min(joy_msg.axes[6], 0)),  # dpad_right
            # Buttons
            "claw":             joy_msg.buttons[0], # a
            "bambi_mode":       joy_msg.buttons[1], # b
            # "":                 joy_msg.buttons[2], # x
            # "":                 joy_msg.buttons[3], # y
            # "":                 joy_msg.buttons[4], # left_bumper
            # "":                 joy_msg.buttons[5], # right_bumper
            # "":                 joy_msg.buttons[6], # window
            # "":                 joy_msg.buttons[7], # menu
            # "":                 joy_msg.buttons[8], # xbox
        }

        # Bambi mode cuts all twist values in half for more precise movements
        bambi_div = 2 if self.__buttons["bambi_mode"].check_state(controller["bambi_mode"]) else 1

        self.get_logger().info(f"{bambi_div}")

        # Create twist message
        twist_msg = Twist()
        twist_msg.linear.x  = controller["linear_x"]     / bambi_div      # Z (forwards)
        twist_msg.linear.y  = -controller["linear_y"]    / bambi_div     # Y (sideways)
        twist_msg.linear.z  = ((controller["neg_linear_z"] - controller["pos_linear_z"]) / 2) / bambi_div # Z (depth)
        twist_msg.angular.x = 0.0 # R (roll) (we don"t need roll)
        twist_msg.angular.y = controller["angular_y"]    / bambi_div     # P (pitch) 
        twist_msg.angular.z = -controller["angular_z"]   / bambi_div     # Y (yaw)
     
        # Publish twist message
        self.__twist_pub.publish(twist_msg)

        # Create claw message
        claw_msg = Bool()
        claw_msg.data = self.__buttons["claw"].check_state(controller["claw"])

        # Publish claw message
        self.__claw_pub.publish(claw_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(InputXboxOne())
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
