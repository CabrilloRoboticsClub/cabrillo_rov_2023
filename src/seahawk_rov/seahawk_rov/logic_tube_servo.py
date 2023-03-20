'''
logic_tube_servo.py

code for controlling the servo hat in the logic tube

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

# ros messages
from std_msgs.msg import Float32

# adafruit servokit for hat
from adafruit_servokit import ServoKit


# linear interpolation helper function
def lerp(old_min:float, old_max:float, new_min:int, new_max:int, old_value:float):
    old_range = old_max - old_min
    new_range = new_max - new_min
    new_value = (((old_value - old_min) * new_range) / old_range) + new_min
    return new_value

# clamp helper function
# helps keep us from starting fires
def clamp(num, minimum, maximum):
  return max(min(minimum, num), maximum)


class LogicTubeServo:
    def __init__(self, node, i2c):
        # instanciate subscribers
        self.drive_cam = node.create_subscription(Float32, 'camera_control', self.receive_drive_camera, 10)

        # drive cam servo is on pin 15
        self.drive_cam_servo = 15

        # instanciate servokit
        self.kit = ServoKit(channels=16, i2c=i2c, address=0x40)

        # configure outputs
        self.kit.servo[self.drive_cam_servo].set_pulse_width_range(0, 3000)
        self.kit.servo[self.drive_cam_servo].actuation_range = 3000
        self.kit.servo[self.drive_cam_servo].angle = 1500


    def receive_drive_camera(self, message):
        self.kit.servo[self.drive_cam_servo].angle = int(lerp(-1.0, 1.0, 0, 3000, clamp(message.data, -1, 1)))
