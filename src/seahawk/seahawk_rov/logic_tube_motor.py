'''
seahawk_rov/logic_tube_motor.py

code for controlling the motor hat in the logic tube

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

# time is needed
import time

# ros messages
from std_msgs.msg import Int8MultiArray

# adafruit circuitpython motor driver
from adafruit_motorkit import MotorKit


class LogicTubeMotor:
    def __init__(self, node, I2C, callback_group):

        # instantiate the claws subscriber
        self.claws = node.create_subscription(Int8MultiArray, 'claw_control', self.receive_claws, 10, callback_group=callback_group)

        # instantiate the motor hat
        self.kit = MotorKit(i2c=I2C, address=0x60)

    def receive_claws(self, message:Int8MultiArray):
        # solenoid 1
        if message.data[0] == 0:
            self.kit.motor1.throttle = None
        else:
            self.kit.motor1.throttle = 1
        
        # solenoid 2
        if message.data[1] == 0:
            self.kit.motor2.throttle = None
        else:
            self.kit.motor2.throttle = 1
        
        # solenoid 3
        if message.data[2] == 0:
            self.kit.motor3.throttle = None
        else:
            self.kit.motor3.throttle = 1
