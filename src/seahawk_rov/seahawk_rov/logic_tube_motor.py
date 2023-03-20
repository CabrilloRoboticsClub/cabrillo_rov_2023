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

# clamp helper function
# helps keep us from starting fires
def clamp(num, minimum, maximum):
  '''clamp helper function'''
  return max(min(minimum, num), maximum)

class LogicTubeMotor:
    def __init__(self, node, I2C):

        # instanciate the claws subscriber
        self.claws = node.create_subscription(Int8MultiArray, 'claw_control', self.recieve_claws, 10)

        # instanciate the motor hat
        self.kit = MotorKit(i2c=I2C, address=0x60)

    def recieve_claws(self, message):
        # solenoid 1
        if message.data[1] == 0:
            self.kit.motor1.throttle = None
        else:
            self.kit.motor1.throttle = 1
        
        # solenoid 2
        if message.data[2] == 0:
            self.kit.motor2.throttle = None
        else:
            self.kit.motor2.throttle = 1
        
        # solenoid 3
        if message.data[3] == 0:
            self.kit.motor3.throttle = None
        else:
            self.kit.motor3.throttle = 1
