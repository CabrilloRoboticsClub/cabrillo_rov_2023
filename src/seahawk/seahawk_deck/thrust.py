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
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from scipy.optimize import curve_fit
import numpy as np

class Thrust(Node):
    """
    Class that implements the kinematics.
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__('thrust')

        self.MAX_FWD_THRUST = 36.3826715 # N
        self.MAX_REV_THRUST = -28.6354180 # N

        self.motor_positions = [
            [0.19, 0.12, 0.047],
            [0.19, -0.12, 0.047],
            [-0.19, 0.12, 0.047],
            [-0.19, -0.12, 0.047],
            [0.105, 0.15, -0.038],
            [0.105, -0.15, -0.038],
            [-0.105, 0.15, -0.038],
            [-0.105, -0.15, -0.038]
        ]

        self.motor_thrusts = [
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.7071, -0.7071, 0],
            [0.7071, 0.7071, 0],
            [-0.7071, -0.7071, 0],
            [-0.7071, 0.7071, 0]
        ]

        self.declare_parameter('center_of_mass_offset', [0.0, 0.0, 0.0])
        self.center_of_mass_offset = self.get_parameter('center_of_mass_offset').value

        self.add_on_set_parameters_callback(self.update_center_of_mass)

        self.motor_config = self.generate_motor_config()
        self.inverse_config = np.linalg.pinv(self.motor_config, rcond=1e-15, hermitian=False)

        self.subscription = self.create_subscription(Twist, 'drive/twist', self._callback, 10)
        self.motor_pub = self.create_publisher(Float32MultiArray, 'drive/motors', 10)
        self.__params = Thrust.__generate_curve_fit_params()

    def update_center_of_mass(self, params: list[Parameter]) -> SetParametersResult:
        """
        Callback for parameter update. Updates the Center of Mass offset and the motor and inverse
        config afterwards.

        Args:
            params: List of updated parameters (handles by ROS2)

        Returns:
            SetParametersResult() which lets ROS2 know if the parameters were set correctly or not
        """
        try:
            self.center_of_mass_offset = params[0]._value.tolist()
        except:
            return SetParametersResult(successful=False)

        self.motor_config = self.generate_motor_config()
        self.inverse_config = np.linalg.pinv(self.motor_config, rcond=1e-15, hermitian=False)

        return SetParametersResult(successful=True)

    def generate_motor_config(self):
        """
        Generate the motor configuration matrix based on motor positions and thrust. Allows for
        a shifting center of mass, so the motor configuration can be regenerated dynamically to
        account for center of mass shifts when lifting objects.

        Returns:
            Motor configuration matrix based on motor orientation, position, and location of center of mass
        """
        motor_shift_lambda = lambda motor: np.subtract(motor, self.center_of_mass_offset).tolist()
        shifted_positons = list(map(motor_shift_lambda, self.motor_positions))
        torques = np.cross(shifted_positons, self.motor_thrusts)

        return [
            list(map(lambda thrust: thrust[0], self.motor_thrusts)), # Fx (N)
            list(map(lambda thrust: thrust[1], self.motor_thrusts)), # Fy (N)
            list(map(lambda thrust: thrust[2], self.motor_thrusts)), # Fz (N)
            list(map(lambda torque: torque[0], torques)),            # Rx (N*m)
            list(map(lambda torque: torque[1], torques)),            # Ry (N*m)
            list(map(lambda torque: torque[2], torques))             # Rz (N*m)
        ]

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

        with open("src/seahawk/seahawk_deck/thrust_to_current.tsv", "r") as file:
            for data_point in file:
                data = data_point.split("\t")
                x.append(data[0])
                y.append(data[1])

        optimal_params, param_covariance = curve_fit(Thrust.__thrust_to_current, x, y)
        return optimal_params

    def get_thrust_limit_scalar(self, motor_values: list) -> float:
        """
        Generate scaling factor based on thrust limitations

        Args:
            motor_values: The motor values in newtons that when produced will result in our desired twist

        Returns:
            Largest scalar the motor values can be scaled by without exceeding thrust limits
        """
        # Scalar is infinite if 0, since there is no limit to how large it can be scaled
        scalar_lambda = lambda x: (self.MAX_FWD_THRUST / x) if x > 0 else ((self.MAX_REV_THRUST / x) if x < 0 else float('inf'))
        return min(map(scalar_lambda, motor_values))

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
        motor_msg.data = list(np.matmul(self.inverse_config, twist_array))

        scalar = self.get_thrust_limit_scalar(motor_msg.data) * max(twist_array)
        motor_msg.data = list(map(lambda x: x * scalar, motor_msg.data))

        self.motor_pub.publish(motor_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Thrust())
    rclpy.shutdown()    


if __name__ == '__main__':
    main(sys.argv)
