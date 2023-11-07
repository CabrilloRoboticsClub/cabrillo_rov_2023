'''
thrust.py

Calculate correct output of motors and output it on /drive/motors

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
import sys 

import rclpy

from rclpy.node import Node 

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

import numpy as np

class ThrustVerify(Node):
    """Caculates expected input, based on current motor status"""

    def __init__(self):
        super().__init__('thrust')

        self.subscription = self.create_subscription(Float32MultiArray, 'drive/motors', self.motor_math, 10)
        self.vector_predict = self.create_publisher(Float32MultiArray, 'drive/predict', 10)

    def motor_math(self, msg):
        # motor 0-7 within list index
        motor_msg = msg

        config = [
            [0, 0, 0, 0, 0.7071, 0.7071, -0.7071, -0.7071], 
            [0, 0, 0, 0, -0.7071, 0.7071, -0.7071, 0.7071], 
            [1, 1, 1, 1, 0, 0, 0, 0],
            [-.19, -.19, .19, .19, -.105, -.105, .105, .105],
            [-.12, .12, -.12, .12, -.15, .15, -.15, .15],
            [-.047,-.047,-.047, -.047, 0.038, 0.038, 0.038, 0.038]
        ]
        
        # Linear Thrust Vector
        x = config[0][0]*motor_msg.data[4] + config[0][1]*motor_msg.data[5] + config[0][2]*motor_msg.data[6] + config[0][3]*motor_msg.data[7]
        y = config[1][0]*motor_msg.data[4] + config[1][1]*motor_msg.data[5] + config[1][2]*motor_msg.data[6] + config[1][3]*motor_msg.data[7]
        z = config[2][0]*motor_msg.data[0] + config[2][1]*motor_msg.data[1] + config[2][2]*motor_msg.data[2] + config[2][3]*motor_msg.data[3]

        # Angular Thrust Vector
        rx = config[3][0]*motor_msg.data[0] + config[3][1]*motor_msg.data[1] + config[3][2]*motor_msg.data[2] + config[3][3]*motor_msg.data[3] + config[3][4]*motor_msg.data[4] + config[3][5]*motor_msg.data[5] + config[3][6]*motor_msg.data[6] + config[3][7]*motor_msg.data[7]
        ry = config[4][0]*motor_msg.data[0] + config[4][1]*motor_msg.data[1] + config[4][2]*motor_msg.data[2] + config[4][3]*motor_msg.data[3] + config[4][4]*motor_msg.data[4] + config[4][5]*motor_msg.data[5] + config[4][6]*motor_msg.data[6] + config[4][7]*motor_msg.data[7]
        rz = config[5][0]*motor_msg.data[0] + config[5][1]*motor_msg.data[1] + config[5][2]*motor_msg.data[2] + config[5][3]*motor_msg.data[3] + config[5][4]*motor_msg.data[4] + config[5][5]*motor_msg.data[5] + config[5][6]*motor_msg.data[6] + config[5][7]*motor_msg.data[7]

        predicted_vectors = Float32MultiArray()
        predicted_vectors.data = [x, y, z, rx, ry, rz]

        self.vector_predict.publish(predicted_vectors)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ThrustVerify())
    rclpy.shutdown()    

if __name__ == '__main__':
    main(sys.argv)
