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
# from threading import Thread

import seahawk_rov

rclpy.init(args=None)

# this creates the node "i2c_proxy"
node_seahawk_rov = rclpy.create_node('seahawk_rov')

# grab the i2c interface for us to use
i2c = board.I2C()

# instnciate the output classes
logic_tube_servo = seahawk_rov.LogicTubeServo(node_seahawk_rov, i2c)
thrust_box_servo = seahawk_rov.ThrustBoxServo(node_seahawk_rov, i2c)

# instanciate the sensor classes
logic_tube_bme280 = seahawk_rov.LogicTubeBME280(node_seahawk_rov, i2c)
# logic_tube_bno085 = LogicTubeBNO085(node_seahawk_rov, i2c)
thrust_box_bme280 = seahawk_rov.ThrustBoxBME280(node_seahawk_rov, i2c)


def main(args=None):

    def publisher():
        logic_tube_bme280.poll()
        logic_tube_bme280.publish()
        # logic_tube_bno085.publish()
        thrust_box_bme280.poll()
        thrust_box_bme280.publish()

    # publish_timer = node_seahawk_rov.create_timer(1, publisher)

    # Threadimg information: See the tasks on git hub for links
    # t = Thread(target=, args=) # target is the function to run, args are the arguements to send to the function
    # t.run() # Runs the function

    rclpy.spin(node_seahawk_rov)

# # # # # # # #
#
# graceful shutdown
#
# # # # # # # #
def signal_handler(sig, frame):

    logic_tube_servo.kit.servo[0].angle = None
    logic_tube_servo.kit.servo[1].angle = None
    logic_tube_servo.kit.servo[2].angle = None
    logic_tube_servo.kit.servo[3].angle = None
    logic_tube_servo.kit.servo[3].angle = None
    logic_tube_servo.kit.servo[4].angle = None
    logic_tube_servo.kit.servo[5].angle = None
    logic_tube_servo.kit.servo[6].angle = None
    logic_tube_servo.kit.servo[7].angle = None
    logic_tube_servo.kit.servo[8].angle = None
    logic_tube_servo.kit.servo[9].angle = None
    logic_tube_servo.kit.servo[10].angle = None
    logic_tube_servo.kit.servo[11].angle = None
    logic_tube_servo.kit.servo[12].angle = None
    logic_tube_servo.kit.servo[13].angle = None
    logic_tube_servo.kit.servo[14].angle = None
    logic_tube_servo.kit.servo[15].angle = None

    thrust_box_servo.kit.servo[0].angle = None
    thrust_box_servo.kit.servo[1].angle = None
    thrust_box_servo.kit.servo[2].angle = None
    thrust_box_servo.kit.servo[3].angle = None
    thrust_box_servo.kit.servo[3].angle = None
    thrust_box_servo.kit.servo[4].angle = None
    thrust_box_servo.kit.servo[5].angle = None
    thrust_box_servo.kit.servo[6].angle = None
    thrust_box_servo.kit.servo[7].angle = None
    thrust_box_servo.kit.servo[8].angle = None
    thrust_box_servo.kit.servo[9].angle = None
    thrust_box_servo.kit.servo[10].angle = None
    thrust_box_servo.kit.servo[11].angle = None
    thrust_box_servo.kit.servo[12].angle = None
    thrust_box_servo.kit.servo[13].angle = None
    thrust_box_servo.kit.servo[14].angle = None
    thrust_box_servo.kit.servo[15].angle = None

    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# # # # # # # #
#
# huh?
#
# # # # # # # #

if __name__ == '__main__':
    main(sys.argv)