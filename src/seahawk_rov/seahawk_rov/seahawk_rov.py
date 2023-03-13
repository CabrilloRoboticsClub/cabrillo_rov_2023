'''
seahawk_rov.py

this is the main node that runs on the ROV

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



def main(args=None):
    rclpy.init(args=args)

    # this creates the node "i2c_proxy"
    node_seahawk_rov = rclpy.create_node('seahawk_rov')

    # grab the i2c interface for us to use
    i2c = board.I2C()

    # instanciate the sensor publishers
    logic_tube_bme280 = LogicTubeBME280(node_seahawk_rov, i2c)
    

    rclpy.spin(node_i2c_proxy)


# # # # # # # #
#
# graceful shutdown
#
# # # # # # # #
def signal_handler(sig, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


if __name__ == '__main__':
    main(sys.argv)