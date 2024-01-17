"""
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
"""
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
        super().__init__("thrust")

        self.MAX_FWD_THRUST = 36.3826715 # N
        self.MAX_REV_THRUST = -28.6354180 # N

        self.TOTAL_CURRENT_LIMIT = 70 # A
        self.ESC_CURRENT_LIMIT = 40 # A

        self.motor_positions = [ # [X, Y, Z] positions for each motors
            [0.19, 0.12, 0.047],    # Motor 0
            [0.19, -0.12, 0.047],   # Motor 1
            [-0.19, 0.12, 0.047],   # Motor 2
            [-0.19, -0.12, 0.047],  # Motor 3
            [0.105, 0.15, -0.038],  # Motor 4
            [0.105, -0.15, -0.038], # Motor 5
            [-0.105, 0.15, -0.038], # Motor 6
            [-0.105, -0.15, -0.038] # Motor 7
        ]

        self.motor_thrusts = [ # [X, Y, Z] components of thrust for each motor
            [0.0, 0.0, 1.0],       # Motor 0
            [0.0, 0.0, 1.0],       # Motor 1
            [0.0, 0.0, 1.0],       # Motor 2
            [0.0, 0.0, 1.0],       # Motor 3
            [0.7071, -0.7071, 0],  # Motor 4
            [0.7071, 0.7071, 0],   # Motor 5
            [-0.7071, -0.7071, 0], # Motor 6
            [-0.7071, 0.7071, 0]   # Motor 7
        ]

        self.declare_parameter("center_of_mass_offset", [0.0, 0.0, 0.0])
        self.center_of_mass_offset = self.get_parameter("center_of_mass_offset").value

        self.add_on_set_parameters_callback(self.update_center_of_mass)

        self.motor_config = self.generate_motor_config()
        self.inverse_config = np.linalg.pinv(self.motor_config, rcond=1e-15, hermitian=False)

        self.subscription = self.create_subscription(Twist, "desired_twist", self._callback, 10)
        self.motor_pub = self.create_publisher(Float32MultiArray, "motor_values", 10)
        self.__params = Thrust.__generate_curve_fit_params()

    def get_polynomial_coef(self, mv: list, limit: float) -> list:
        """
        Generates a list of the coefficients for a polynomial, the input of which is the
        motor scaling factor and the roots of the function are the maximum scaling factor.
        Args:
            mv: The motor values in newtons that when produced will result in our desired twist
            limit: The current limit we would like to stay under in amperes (TOTAL_CURRENT_LIMIT or ESC_CURRENT_LIMIT)

        Returns:
            A list of the coefficients of a 5th degree polynomial function, where the input of said
            function is the scaling factor and the output is the current (A) draw
        """
        return [self.__params[0] * sum([thrust**6 for thrust in mv]),
                self.__params[1] * sum([thrust**5 for thrust in mv]),
                self.__params[2] * sum([thrust**4 for thrust in mv]),
                self.__params[3] * sum([thrust**3 for thrust in mv]),
                self.__params[4] * sum([thrust**2 for thrust in mv]),
                self.__params[5] * sum(mv),
                self.__params[6] * len(mv) - limit]

    def get_current_scalar_value(self, mv: list, limit: float) -> float:
        """
        Given a motor value list and a current limit, return the best scaling factor

        Args:
            mv: The motor values in newtons that when produced will result in our desired twist
            limit: The current limit we would like to stay under in amperes (TOTAL_CURRENT_LIMIT or ESC_CURRENT_LIMIT)

        Returns:
            A valid scaling factor
        """
        # Get coefficients for function given the motor values given and the current (Amp) limits
        coef_list = self.get_polynomial_coef(mv, limit)
        # Find roots
        potential_scaling_factors = np.roots(coef_list).tolist()
        # Ignore nonreal and negative scaling factors
        real_positive = [scalar.real for scalar in potential_scaling_factors if scalar.imag == 0 and scalar.real >= 0]
        # Return valid roots
        return min(real_positive)

    def get_minimum_current_scalar(self, mv: list) -> float:
        """
        Returns a scalar which shows the maximum amount of thrust the robot can produce for the
        given direction without exceeding total current (A) limits, or the current (A) limit of
        either ESC

        Args:
            mv: The motor values in newtons that when produced will result in our desired twist

        Returns:
            The largest scalar we can scale those motor values by without exceeding the total current
            (A) limit and the current limit of each ESC
        """
        # All motors
        total_scalar = self.get_current_scalar_value(mv, self.TOTAL_CURRENT_LIMIT)
        # First four motors / motors on esc 1
        esc1_scalar = self.get_current_scalar_value(mv[:4], self.ESC_CURRENT_LIMIT)
        # Second four motors / motors on esc 2
        esc2_scalar = self.get_current_scalar_value(mv[4:], self.ESC_CURRENT_LIMIT)

        return min(total_scalar, esc1_scalar, esc2_scalar)



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
        shifted_positons = [(np.subtract(motor, self.center_of_mass_offset).tolist())
                            for motor in self.motor_positions]
        torques = np.cross(shifted_positons, self.motor_thrusts)

        return [
            [thrust[0] for thrust in self.motor_thrusts], # Fx (N)
            [thrust[1] for thrust in self.motor_thrusts], # Fy (N)
            [thrust[2] for thrust in self.motor_thrusts], # Fz (N)
            [torque[0] for torque in torques],            # Rx (N*m)
            [torque[1] for torque in torques],            # Ry (N*m)
            [torque[2] for torque in torques]             # Rz (N*m)
        ]

    @staticmethod
    def __thrust_to_current(x: float, a: float, b: float, c: float, d: float, e: float, f: float, g: float) -> float:
        """
        Estimates current draw based on given thrust

        Args:
            x: Thrust being produced in newtons.
            a-f: Arbitrary parameters to map thrust to current, see __generate_curve_fit_params()

        Returns:
            Current (estimated) to be drawn in amps.
        """
        return (a * x**6) + (b * x**5) + (c * x**4) + (d * x**3) + (e * x**2) + (f * x) + (g)

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
        return min([(self.MAX_FWD_THRUST / thrust) if thrust > 0
                    else ((self.MAX_REV_THRUST / thrust) if thrust < 0
                        else float("inf"))
                    for thrust in motor_values])

    def _callback(self, twist_msg):
        """Called every time the twist publishes a message."""

        # Convert the X,Y,Z,R,P,Y to thrust settings for each motor. 
        motor_msg = Float32MultiArray()

        # Convert Twist to single vector for multiplication
        twist_array = [
            twist_msg.linear.x,
            twist_msg.linear.y,
            twist_msg.linear.z,
            twist_msg.angular.x,
            twist_msg.angular.y,
            twist_msg.angular.z
        ]

        if twist_array == [0, 0, 0, 0, 0, 0]:
            motor_msg.data = [0.0 for motor in range(8)] # No thrust needed
            self.motor_pub.publish(motor_msg)
            return

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

        # Multiply twist with inverse of motor config to get motor effort values
        motor_msg.data = np.matmul(self.inverse_config, twist_array).tolist()

        thurst_scalar = self.get_thrust_limit_scalar(motor_msg.data)
        current_scalar = self.get_minimum_current_scalar(motor_msg.data)
        # Scalar will be the smaller of the two, largest value in twist array
        # will be percentage of that maximum
        scalar = min(thurst_scalar, current_scalar) * max(twist_array)

        # scale motor values
        motor_msg.data = [thrust * scalar for thrust in motor_msg.data]

        self.motor_pub.publish(motor_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Thrust())
    rclpy.shutdown()    


if __name__ == "__main__":
    main(sys.argv)
