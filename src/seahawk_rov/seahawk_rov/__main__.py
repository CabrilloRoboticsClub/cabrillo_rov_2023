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

# threading imports
import sys
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import seahawk_rov

def main(args=None):
    rclpy.init(args=args)

    # this creates the node "i2c_proxy"
    try:
        # Runs all callbacks in the main thread
        executor = MultiThreadedExecutor(num_threads=4)

        # Callback groups
        fast_group = MutuallyExclusiveCallbackGroup() # pwm boards
        slow_group = MutuallyExclusiveCallbackGroup() # sensors
        imu_group = MutuallyExclusiveCallbackGroup() # imu for independent control

        # Add imported nodes to this executor
        node_seahawk_rov = rclpy.create_node('seahawk_rov')
        executor.add_node(node_seahawk_rov)

        # Grab the i2c interface for us to use
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)

        # instantiate the output classes
        logic_tube_servo = seahawk_rov.LogicTubeServo(node_seahawk_rov, i2c, fast_group)
        logic_tube_servo_shutdown = rclpy.get_default_context().on_shutdown(logic_tube_servo.on_shutdown)

        logic_tube_motors = seahawk_rov.LogicTubeMotor(node_seahawk_rov, i2c, fast_group)
        
        thrust_box_servo = seahawk_rov.ThrustBoxServo(node_seahawk_rov, i2c, fast_group)
        thrust_box_servo_shutdown = rclpy.get_default_context().on_shutdown(thrust_box_servo.on_shutdown)

        # setup the logic tube bme280
        logic_tube_bme280 = seahawk_rov.BME280(
            node = node_seahawk_rov,
            i2c_bus = i2c,
            i2c_addr = 0x77,
            frame_id = "base_link",
            hardware_location = "logic_tube"
        )
        logic_tube_bme280_publish_timer = node_seahawk_rov.create_timer(5, logic_tube_bme280.publish, callback_group=slow_group)

        # setup the logic tube bno085
        logic_tube_bno085 = seahawk_rov.BNO085(
            node = node_seahawk_rov,
            i2c_bus = i2c,
            i2c_addr = 0x4a,
            frame_id = "logic_tube_bno085",
            hardware_location = 'logic_tube'
        ) 
        logic_tube_imu_publish_timer = node_seahawk_rov.create_timer(0.1, logic_tube_bno085.publish, callback_group=imu_group)

        # setup the thrust box bme280
        thrust_box_bme280 = seahawk_rov.BME280(
            node = node_seahawk_rov,
            i2c_bus = i2c,
            i2c_addr = 0x76,
            frame_id = "base_link",
            hardware_location = "thrust_box"
        )
        thrust_box_bme280_publish_timer = node_seahawk_rov.create_timer(5, thrust_box_bme280.publish, callback_group=slow_group)

        try:
            # Execute callbacks nodes as they become ready
            executor.spin()
        finally:
            executor.shutdown()
            node_seahawk_rov.destroy_node()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()


# # # # # # # #
#
# graceful shutdown
#
# # # # # # # #
# def signal_handler(sig, frame):
#     sys.exit(0)

# signal.signal(signal.SIGINT, signal_handler)

# # # # # # # #
#
# huh?
#
# # # # # # # #

if __name__ == '__main__':
    main(sys.argv)