"""
input_keyboard.py

Take input from the keyboard and republish it to topic 'keystroke'

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
# ROS client library
import rclpy
from rclpy.node import Node 

# ROS messages
from std_msgs.msg import String

# Threading
import threading

# For keyboard stuff
import sys, tty, os, termios

# Handle requesting to set another node's parameters
from .set_remote_params import SetRemoteParams

class InputKeyboard(Node):
    """
    Class that actively reads keyboard input
    """

    def __init__(self, settings):
        """
        Initialize 'input_keyboard' node
        """
        super().__init__("input_keyboard")
    
        # Create publisher to topic 'key_press'
        self.__key_pub = self.create_publisher(String, "keystroke", 10)

        # Set up client to remotely set parameters on 'input_xbox_one' node using a service
        self.__set_params_input_xbox_one = SetRemoteParams(self, "input_xbox_one")

        # Get current user terminal settings and save them for later
        self.__settings = settings

        # Store the current key
        self.__cur_key = ""
    
    def __get_key(self) -> str:
        """
        Extracts a single keystroke from the user and sets private attribute __cur_key
        """
        # Enable raw mode. In raw mode characters are directly read from and written 
        # to the device without any translation or interpretation by the operating system
        tty.setraw(sys.stdin.fileno())

        # Read at most one byte (the length of one character) from stdin and decode it
        self.__cur_key = os.read(sys.stdin.fileno(), 1).decode()

        # Make terminal not break
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.__settings)

    def callback(self):
        """
        Gets a key from the user publishes it to 'keystroke' topic and update any parameters

        Throws 'KeyboardInterrupt' if user enters ctrl-c
        """
        self.__get_key()

        # If 'ctrl-c' raise 'KeyboardInterrupt'
        if self.__cur_key == "\x03":
            raise KeyboardInterrupt("Terminated the process with ctrl-c")
        
        self.__pub_callback()
        self.__param_callback()

    def __pub_callback(self):
        """
        Publishes the current key to 'keystroke' topic

        Throws 'KeyboardInterrupt' if user enters ctrl-c
        """
        # Create and publish message
        msg = String()
        msg.data = self.__cur_key
        self.__key_pub.publish(msg)
    
    def __param_callback(self):
        """
        Updates parameters for 'throttle_curve_choice'
        """
        # If the user pressed '0', '1', or '2' send that information to the __set_params_input_xbox_one
        if self.__cur_key in ["0", "1", "2"]:
            self.__set_params_input_xbox_one.update_params("throttle_curve_choice", self.__cur_key)

        # Send all params to input_xbox_one node
        self.__set_params_input_xbox_one.send_params()

def main(args=None):
    rclpy.init(args=args)

    # Get current user terminal settings and save them for later
    orig_settings = termios.tcgetattr(sys.stdin)

    # Instance of InputKeyboard()
    node = InputKeyboard(orig_settings)

    # Threading allows the process to look for input and run the node at the same time
    # Create and start a thread for spinning the node
    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    try:
        while True:
            # While the user has not entered 'ctrl-c', wait for keystrokes and publish them
            # to `keystroke`. If the user enters a key mapped to a parameter, update that parameter
            node.callback()
    except KeyboardInterrupt as error_msg:
        print(error_msg)
    
    # Kill node
    rclpy.shutdown()

    # Delays a program's flow of execution until spinner is finished its process
    spinner.join()
    
    # Reset to original terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)

    # Make terminal sane again upon program exit
    os.system("stty sane")


if __name__ == "__main__":
    main(sys.argv)
