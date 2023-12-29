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
from scipy.optimize import curve_fit

import numpy as np

class Thrust(Node):
    """
    Class that implements the kinematics.
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__('thrust')        

        self.motor_config = [
            [     0,     0,    0,     0,     0.7071,     0.7071,    -0.7071,   -0.7071 ],  # Fx (N)
            [     0,     0,    0,     0,    -0.7071,     0.7071,    -0.7071,    0.7071 ],  # Fy (N)
            [     1,     1,    1,     1,          0,          0,          0,         0 ],  # Fz (N)
            [  0.12, -0.12, 0.12, -0.12, -0.0268698,  0.0268698, -0.0268698, 0.0268698 ],  # Rx (N*m)
            [ -0.19, -0.19, 0.19,  0.19, -0.0268698, -0.0268698,  0.0268698, 0.0268698 ],  # Ry (N*m)
            [     0,     0,    0,     0,  -0.180311,   0.180311,   0.180311, -0.180311 ],  # Rz (N*m)
        ]

        self.inverse_config = np.linalg.pinv(self.motor_config, rcond=1e-15, hermitian=False)

        self.subscription = self.create_subscription(Twist, 'drive/twist', self._callback, 10)
        self.motor_pub = self.create_publisher(Float32MultiArray, 'drive/motors', 10)
        self.__params = Thrust.__generate_curve_fit_params()
        
    @staticmethod
    def __thrust_to_current(x: float, a: float, b: float, c: float, d: float, e: float, f: float) -> float:
        """
        Estimates current draw based on given thrust

        Args:
            x: Thrust being produced in newtons.
            a-f: Arbitrary parameters to map thrust to current, see __generate_curve_fit_params()

        Returns:
            Current (estimated) to be drawn in amps.
        """
        return (a * x**5) + (b * x**4) + (c * x**3) + (d * x**2) + (e * x) + (f)

    @staticmethod
    def __generate_curve_fit_params() -> list:
        """
        Generates Optimal Parameters for __thrust_to_current() to have a best fit

        Returns:
            List of optimal parameters
        """
        x = list()
        y = list()

        with open("thrust_to_current.tsv", "r") as file:
            for data_point in file:
                data = data_point.split("\t")
                x.append(data[0])
                y.append(data[1])

        optimal_params, param_covariance = curve_fit(Thrust.__thrust_to_current, x, y)
        return optimal_params


    def _callback(self, twist_msg):
        """Called every time the twist publishes a message."""

        # Convert the X,Y,Z,R,P,Y to thrust settings for each motor. 
        motor_msg = Float32MultiArray()
        # +1 = Full thrust, Forwards
        #  0 = Off
        # -1 = Full thrust, Backwards
        # Even are on bottom
        # 45° angle(π/4)
        # ^FRONT^
        # 6/. .\0
        # 4\^ ^/2
        # Odd are on top; thrust forward is up
        # 35° angle(7π/36)
        # ^FRONT^
        #  7   1
        #  5   3

        # Convert Twist to single vector for multiplication
        twist_array = [
            twist_msg.linear.x,
            twist_msg.linear.y,
            twist_msg.linear.z,
            twist_msg.angular.x,
            twist_msg.angular.y,
            twist_msg.angular.z
        ]

        motor_msg.data = [
            0.0,  # Motor 0 thrust
            0.0,  # Motor 1 thrust
            0.0,  # Motor 2 thrust
            0.0,  # Motor 3 thrust
            0.0,  # Motor 4 thrust
            0.0,  # Motor 5 thrust
            0.0,  # Motor 6 thrust
            0.0,  # Motor 7 thrust
        ]

        # Multiply twist with inverse of motor config to get motor effort values
        motor_efforts = np.matmul(self.inverse_config, twist_array)

        motor_msg.data[0] = motor_efforts[0]
        motor_msg.data[1] = motor_efforts[1]
        motor_msg.data[2] = motor_efforts[2]
        motor_msg.data[3] = motor_efforts[3]
        motor_msg.data[4] = motor_efforts[4]
        motor_msg.data[5] = motor_efforts[5]
        motor_msg.data[6] = motor_efforts[6]
        motor_msg.data[7] = motor_efforts[7]

        self.motor_pub.publish(motor_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Thrust())
    rclpy.shutdown()    


if __name__ == '__main__':
    main(sys.argv)
