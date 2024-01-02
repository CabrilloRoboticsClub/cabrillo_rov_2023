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

from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue, ParameterType

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

        # Set up service to remotely update 'throttle_curve_choice' parameter on 'input_xbox_one' node
        self.__cli_throttle_curve_choice = self.create_client(SetParameters, "/input_xbox_one/throttle_curve_choice")
        while not self.__cli_throttle_curve_choice.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("throttle_curve_choice service not available, waiting again...")
        self.req = SetParameters.Request()

        # Get current user terminal settings and save them for later
        self.__settings = settings
        
        # Store the current key
        self.__cur_key
    
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
        if self.__cur_key in ["0", "1", "2"]:
            new_param_value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=self.__cur_key)
            self.req.parameters = [Parameter(name="throttle_curve_choice", value=new_param_value)]
            self.future = self.__cli_throttle_curve_choice.call_async(self.req)


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
