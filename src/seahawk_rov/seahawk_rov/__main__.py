'''
seahawk_rov/main.py

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

    # this creates the node "i2c_proxy"
    node_seahawk_rov = rclpy.create_node('seahawk_rov')

    # grab the i2c interface for us to use
    i2c = board.I2C()

    # instnciate the output classes
    logic_tube_servo = seahawk_rov.LogicTubeServo(node_seahawk_rov, i2c)
    logic_tube_motors = seahawk_rov.LogicTubeMotor(node_seahawk_rov, i2c)
    thrust_box_servo = seahawk_rov.ThrustBoxServo(node_seahawk_rov, i2c)

    # instanciate the sensor classes
    logic_tube_bme280 = seahawk_rov.LogicTubeBME280(node_seahawk_rov, i2c)
    # logic_tube_bno085 = LogicTubeBNO085(node_seahawk_rov, i2c)
    thrust_box_bme280 = seahawk_rov.ThrustBoxBME280(node_seahawk_rov, i2c)
    
    def publisher():
        logic_tube_bme280.publish()
        # logic_tube_bno085.publish()
        thrust_box_bme280.publish()

    publish_timer = node_seahawk_rov.create_timer(0.1, publisher)

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