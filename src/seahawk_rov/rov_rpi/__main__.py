'''
rov_rpi/__main__.py

the main for the node that access the rpi gpio on the rov

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

# # # # # # # # 
#
# IMPORTS
#
# # # # # # # #

# time is needed
import time

# enable command line arguments
import sys
# enable signal handling
import signal

# ros stuff
import rclpy
from rclpy.node import Node

# library for accessing the raspberry pi board
# aka /dev/i2c
import board
import busio

import seahawk_rov

def main(args=None):
    rclpy.init(args=args)

    # grab the i2c interface for us to use
    i2c = board.I2C()


    # this creates the node "seahawk_rov"
    node_seahawk_rov = rclpy.create_node('seahawk_rov_rpi')

    # instnciate the output classes
    logic_tube_servo = seahawk_rov.LogicTubeServo(node_seahawk_rov, i2c)
    logic_tube_motors = seahawk_rov.LogicTubeMotor(node_seahawk_rov, i2c)
    thrust_box_servo = seahawk_rov.ThrustBoxServo(node_seahawk_rov, i2c)

    rclpy.spin(node_seahawk_rov)

# # # # # # # #
#
# graceful shutdown
#
# # # # # # # #
def signal_handler(sig, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# # # # # # # #
#
# huh?
#
# # # # # # # #

if __name__ == '__main__':
    main(sys.argv)