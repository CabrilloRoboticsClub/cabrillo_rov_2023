'''
seahawk_rov/thrust_box_servo.py

code for controlling the servo board in the thrust box

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

# ros message
from std_msgs.msg import Float32MultiArray

# adafruit circuitpython servo driver
from adafruit_servokit import ServoKit

# import helper functions
import  seahawk_rov

class ThrustBoxServo:
    def __init__(self, node, i2c, callback_group):
        # subscribe to the motors
        self.thrusters = node.create_subscription(Float32MultiArray, 'drive/motors', self.receive_thruster, 10, callback_group=callback_group)

        # map output channels        
        self.thruster_map = (0,1,2,3,4,5,6,7)
        self.pwm_offset = -30 # offset in microseconds

        # instantiate outputs
        self.kit = ServoKit(channels=16, i2c=i2c, address=0x41)

        # configure outputs
        for thruster in self.thruster_map:
            self.kit.continuous_servo[thruster].set_pulse_width_range(1220 + self.pwm_offset, 1780 + self.pwm_offset)


    def receive_thruster(self, message:Float32MultiArray):
        for thruster in self.thruster_map: # for loop to step through thruster map 
            self.kit.continuous_servo[thruster].throttle = message.data[thruster] # set the thruster throttle to the value from the message data

    def on_shutdown(self):
        for thruster in self.thruster_map: # for loop to step through thruster map 
            self.kit.continuous_servo[thruster].throttle = None # set the thruster throttle to None to disable the output