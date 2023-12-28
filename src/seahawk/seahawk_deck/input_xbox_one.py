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
# from std_msgs.msg import Int8MultiArray
# from std_msgs.msg import Float32
# from rclpy.parameter import Parameter

ON = 1
OFF = 0

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
        
        self.__button_state = {
            # "" :                OFF,        # left_stick_press
            # "" :                OFF,        # right_stick_press
            "claw":             {"prev_button_state": OFF, "feature_state": OFF},       # a
            "bambi_mode":       {"prev_button_state": OFF, "feature_state": OFF}        # b
            # "":                 OFF,        # x
            # "":                 OFF,        # y
            # "":                 OFF,        # left_bumper
            # "":                 OFF,        # right_bumper
            # "":                 OFF,        # window
            # "":                 OFF,        # menu
            # "":                 OFF,        # xbox
        }
    
    @staticmethod
    def __throttle_curve(input: float, curve: int=0):
        """
        Applies a throttle curve mapping to the 'input'. A throttle curve allows the user to
        modify the relationship between the stick position and the actual throttle value sent
        to the motors

        The throttle curve is selected by passing a number [0, 3] to the 'curve' param
            0 (default): No throttle curve
            1: 
            2: 
            3: 

        Args:
            input: The value to remap
            curve: The type of curve to remap to

        Returns:
            'input' remapped to fit the specified throttle curve
        """
        match curve:
            case 1:
                pass
            case 2:
                pass
            case 3:
                pass
            case _:
                return input
    

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
            "linear_y":         joy_msg.axes[0],               # left_stick_x
            "linear_x":         joy_msg.axes[1],               # left_stick_y
            # "":                 joy_msg.buttons[9],             # left_stick_press
            # Right stick
            "angular_z":        joy_msg.axes[3],               # right_stick_x
            "angular_y":        joy_msg.axes[4],               # right_stick_y
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


        def sticky_button(button: str) -> bool:
            """
            Makes a button sticky meaning that meaning a button is pressed to turn it on, 
            and pressed again to turn it off

            Args:
                button: The name of the buttom

            Returns:
                True if the feature the button controlls should be turned on, False if off
            """
            if controller[button] == ON and self.__button_state[button]["prev_button_state"] == OFF:
                self.__button_state[button]["feature_state"] = not self.__button_state[button]["feature_state"]
            return bool(self.__button_state[button]["feature_state"])


        # Bambi mode cuts all twist values in half for more precise movements
        bambi_div = 2 if sticky_button("bambi_mode") else 1

        # Create twist message
        twist_msg = Twist()
        twist_msg.linear.x  = controller["linear_x"]     / bambi_div     # Z (forwards)
        twist_msg.linear.y  = -controller["linear_y"]    / bambi_div     # Y (sideways)
        twist_msg.linear.z  = ((controller["neg_linear_z"] - controller["pos_linear_z"]) / 2) / bambi_div # Z (depth)
        twist_msg.angular.x = 0.0 # R (roll) (we don"t need roll)
        twist_msg.angular.y = controller["angular_y"]    / bambi_div     # P (pitch) 
        twist_msg.angular.z = -controller["angular_z"]   / bambi_div     # Y (yaw)
     
        # Publsih twist message
        self.__twist_pub.publish(twist_msg)

        # Create claw message
        claw_msg = Bool()
        claw_msg.data = sticky_button("claw")

        # Publish claw message
        self.__claw_pub.publish(claw_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(InputXboxOne())
    rclpy.shutdown()
   

if __name__ == "__main__":
    main(sys.argv)
