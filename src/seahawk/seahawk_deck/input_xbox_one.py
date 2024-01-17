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
# For reading argv
import sys 
# for reading in command line arguments in python

# Ros client libary imports
import rclpy
from rclpy.node import Node 
# essential improts for creating ROS node functionality

# ROS messages imports
from geometry_msgs.msg import Twist 
# imports messages of type Twist from the geometry_msgs module
from sensor_msgs.msg import Joy
# imports joystick messages of type Joy from sensor_msgs module
from std_msgs.msg import Bool
# imports Bool type messages from std_msgs module

# Q: What is a module?

class StickyButton():
    """
    Class that implements sticky buttons, meaning a button is pressed to turn it on, 
    and pressed again to turn it off
    """

    def __init__(self):
        """
        Initialize 'StickyButton' object
        """
        self.__feature_state = False    # Is the feature this button controls on (True) or off (False)
        self.__track_state = 0b0000     # Tracks the last four states of the button using bits
    
    def check_state(self, cur_button_state: bool) -> bool:
        """
        Checks if a button is toggled on or off and accounts for debouncing

        Args:
            cur_button_state: Current state of the button provided by the controller

        Returns:
            True if the button is toggled on, False if off
        """
        # Append the current button state to the tracker then remove the leftmost bit
        # such that self.__track_state records the most recent four states of the button
        self.__track_state = (self.__track_state << 1 | cur_button_state) & 0x0F

        # Account for bounce by making sure the last four recorded states
        # appear to represent a button press. If so, update the feature state
        if self.__track_state == 0b0011:
            self.__feature_state = not self.__feature_state
        return self.__feature_state

    def reset(self):
        """
        Resets button state to original configuration
        """
        self.__feature_state = False
        self.__track_state = 0b0000

# Q: So are we creating a node inside of a node here?

class InputXboxOne(Node):  # creates a subclass InputXboxOne of superclass Node from ROS2 library
    """
    Class that implements the joystick input
    """

    def __init__(self):  # default construcutor for current instance of InputXboxOne
    # Q: why do we put 'def' before functions? is it like typename <T> from C++?
        """
        Initialize 'input_xbox_one' node
        """
        super().__init__("input_xbox_one")
        # initializes the input_xbox_one node by sending name of node to super class Node
        # Q: Why do we do this? Isnt class InputXboxOne(Node); enough?

        self.subscription = self.create_subscription(Joy, "joy", self.__callback, 10)
        # creates sub for current instance of class with self.create_subscription
        # 'Joy' represents the message type and "joy" is topic from which joystick messages are published
        # self.__callback is causing this to refresh whenever theres a new joystick message
        # 10 is a QoS setting for the sub
        # PURPOSE: this is designed to recieve joystick data from xbox_one controller

        self.__twist_pub = self.create_publisher(Twist, "desired_twist", 10)
        # sets up a publisher message for the current instance of the class
        # publishes Twist type messages to the desired_twist topic 
        # 10 is the QoS setting for the pub
        # PURPOSE: Not sure...

        self.__claw_pub = self.create_publisher(Bool, "claw_state", 10)
        # sets up a publisher message for the current instance of the class
        # publishes Bool type messages to the claw_state topic 
        # 10 is the QoS setting for the pub
        # PURPOSE: this published information about whether or not the claw is open or closed
        
        self.__buttons = {
            # "" :              StickyButton(),     # left_stick_press
            # "" :              StickyButton(),     # right_stick_press
            "claw":             StickyButton(),     # a
            "bambi_mode":       StickyButton()      # b
            # "":               StickyButton(),     # x
            # "":               StickyButton(),     # y
            # "":               StickyButton(),     # window
            # "":               StickyButton(),     # menu
        }
        # a dictionary of key value pairs for mapping out relevant buttons on the controller
        # each key is a string representing a button and its value is an instance of StickyButton class
        # PURPOSE: keep track of whether or not a button is pressed or not
        # Q: how does the StickyButton class work?

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
            # "":               joy_msg.buttons[9],             # left_stick_press
            # Right stick
            "angular_z":        joy_msg.axes[3],                # right_stick_x
            "angular_y":        joy_msg.axes[4],                # right_stick_y
            # "":               joy_msg.buttons[10],            # right_stick_press
            # Triggers
            "neg_linear_z":     joy_msg.axes[2],                # left_trigger
            "pos_linear_z":     joy_msg.axes[5],                # right_trigger
            # Dpad
            # "":               int(max(joy_msg.axes[7], 0)),   # dpad_up
            # "":               int(-min(joy_msg.axes[7], 0)),  # dpad_down
            # "":               int(max(joy_msg.axes[6], 0)),   # dpad_left     
            # "":               int(-min(joy_msg.axes[6], 0)),  # dpad_right
            # Buttons
            "claw":             joy_msg.buttons[0], # a
            "bambi_mode":       joy_msg.buttons[1], # b
            # "":               joy_msg.buttons[2], # x
            # "":               joy_msg.buttons[3], # y
            "pos_angular_x":    joy_msg.buttons[4], # left_bumper
            "neg_angular_x":    joy_msg.buttons[5], # right_bumper
            # "":               joy_msg.buttons[6], # window
            # "":               joy_msg.buttons[7], # menu
            "reset":            joy_msg.buttons[8], # xbox
        }

        # Bambi mode cuts all twist values in half for more precise movements
        bambi_div = 2 if self.__buttons["bambi_mode"].check_state(controller["bambi_mode"]) else 1

        # Create twist message
        twist_msg = Twist()
        twist_msg.linear.x  = controller["linear_x"]     / bambi_div     # Z (forwards)
        twist_msg.linear.y  = -controller["linear_y"]    / bambi_div     # Y (sideways)
        twist_msg.linear.z  = ((controller["neg_linear_z"] - controller["pos_linear_z"]) / 2) / bambi_div # Z (depth)

        # Roll is activated at a constant 0.5 throttle if either the left or right button is pressed
        roll_dir = -1 if controller["neg_angular_x"] else 1
        twist_msg.angular.x = (roll_dir * 0.5 / bambi_div) if controller["pos_angular_x"] or controller["neg_angular_x"] else 0.0 # R (roll)
    
        twist_msg.angular.y = controller["angular_y"]    / bambi_div     # P (pitch) 
        twist_msg.angular.z = -controller["angular_z"]   / bambi_div     # Y (yaw)
     
        # Publish twist message
        self.__twist_pub.publish(twist_msg)

        # Create claw message
        claw_msg = Bool()
        claw_msg.data = self.__buttons["claw"].check_state(controller["claw"])

        # Publish claw message
        self.__claw_pub.publish(claw_msg)

        # If the x-box button is pressed, all settings get reset to default configurations
        if controller["reset"]:
            self.__buttons["bambi_mode"].reset()


def main(args=None):  # args = None means there are no command line arguments
    rclpy.init(args=args)  # ROS initializer for nodes
    rclpy.spin(InputXboxOne())  # keep InputXboxOne node running for as long as needed
    rclpy.shutdown()  # shut down the node when not needed


if __name__ == "__main__":
    main(sys.argv)
# Q: what is this? ^
