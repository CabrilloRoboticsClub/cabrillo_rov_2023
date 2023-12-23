'''
motor_encoding.py

Class that converts newtons to pwm values then to dshot packets

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

# ROS2 imports
import rclpy
from rclpy.node import Node 

# Import ros message types
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray

from scipy.optimize import curve_fit


class Motor_encoding(Node):
    """
    Class that converts newtons to pwm values then to DSHOT packets
    """

    def __init__(self):
        """
        Initialize 'motor_encoding' node
        """
        super().__init__("motor_encoding")
        self.subscription = self.create_subscription(Float32MultiArray, "drive/motors", self._callback, 10)
        self.motor_pub = self.create_publisher(Int16MultiArray, "motor_msgs", 10)
        self.params = self._generate_curve_fit_params()

    @staticmethod
    def _newtons_to_pwm(x:float, a:float, b:float, c:float, d:float, e:float, f:float)->int:
        """
        Converts desired newtons into its corresponding PWM value.

        Parameters:
            x: The force in newtons desired
            a-f: Arbitrary parameters to map newtons to pwm, see _generate_curve_fit_params()

        Returns:
            PWM value corresponding to the desired thrust
        """
        return int((a * x**5) + (b * x**4) + (c * x**3) + (d * x**2) + (e * x) + f)
    
    @staticmethod
    def _generate_curve_fit_params(self)->list:
        """
        Generates Optimal Parameters for _newtowns_to_pwm() to have a best fit

        Returns:
            List of optimal parameters
        """
        x, y = list()

        with open("newtons_to_pwm.tsv", "r") as file:
            for line in file:
                words = line.split("\t")
                x.append(words[0])
                y.append(words[1])
        
        optimal_params, param_covariance = curve_fit(Motor_encoding._newtons_to_pwm, x, y)
        return optimal_params
    
    # TODO
    @staticmethod
    def _pwm_to_dshot(pwm:int, telemetry:bool = False)->int:
        """
        Generates a DSHOT packet given newtons and a telemetry request

        Frames are organized in the following 16 bit pattern: SSSSSSSSSSSTCCCC
            (S) 11 bit throttle
            (T) 1 bit telemetry request
            (C) 4 bit Cyclic Redundancy Check (CRC) (calculated in this function)

        Parameters:
            pwm: A thrust value in newtons to be sent to a motor
            telemetry=False: Telemetry value (1 bit), True (1) if telemetry should be used, False (0) if not

        Returns:
            A 16 bit package of data to send following the pattern SSSSSSSSSSSTCCCC
        """
        # ***********Convert pwm to dshot***********
        
        # If you enable 3D mode, the throttle ranges split in two:
        #   Direction 1) 48 is the slowest, 1047 is the fastest
        #   Direction 2) 1049 is the slowest, 2047 is the fastest
        DIR_1_LOW = 48
        DIR_1_HIGH = 1047
        DIR_2_LOW = 1049
        DIR_2_HIGH = 2047

        # 1048 does NOT stop the motor, the following constant should be used instead
        # Note: One website says this command is not currently implemented, this needs to be tested TODO

        # ************Encode dshot value************
        throttle = None
        # Shift everything over by one bit then append telemetry
        data = (throttle << 1) | telemetry
        # CRC calculation
        crc = (data ^ (data >> 4) ^ (data >> 8)) & 0x0F
        # Shift everything over by 4 bits then append crc
        return (data << 4) | crc

    def _callback(self, kine_msg:Float32MultiArray)->None:
        """
        Converts newtons in kine_msg to PWM values and republishes them.

        Parameters:
            kine_msg: Message from the kinematics node
        """
        # Each DSHOT package is 16 bits
        motor_msg = Int16MultiArray()
    
        # For every newton value provided by kinematics, convert it to pwm then to a dshot package
        for index, newton in enumerate(kine_msg.data):
            motor_msg.data[index] = Motor_encoding._pwm_to_dshot(
                Motor_encoding._newtons_to_pwm(
                    newton,
                    self.params[0],
                    self.params[1],
                    self.params[2],
                    self.params[3],
                    self.params[4],
                    self.params[5]))

        # Publish the encoded motor values to 'motor_msgs' topic
        self.motor_pub.publish(motor_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Motor_encoding())
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
