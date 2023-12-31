#!/usr/bin/env python3
'''
input_keyboard.py

Take input from the keyboard and republish it to topic

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
# ROS client library imports
import rclpy
from rclpy.node import Node 

# ROS messages
from std_msgs.msg import String
import threading

# For keyboard stuff
import sys, tty, os, termios


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    # Get current user terminal settings and save them for later
    orig_settings = termios.tcgetattr(sys.stdin)

    # Initialize node `input_keyboard`
    rclpy.init()
    node = rclpy.create_node("input_keyboard")

    # Create publisher to topic `key_press`
    pub = node.create_publisher(String, "key_press", 10)

    # Begin threading for some fuckin reason
    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    # Initialize message as a String ROS message type
    msg = String()

    try:
        while True:
            # Get the key entered by the user
            key = get_key(orig_settings)

            # Create and publish message
            msg.data = str(key)
            pub.publish(msg)
            print(msg.data)
    except Exception as error_msg:
        node.get_logger().info(error_msg)
    finally:
        rclpy.shutdown()
        spinner.join()
        # Reset to original terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)

        # Make terminal sane again upon program exit
        os.system("stty sane")

if __name__ == "__main__":
    main(sys.argv)