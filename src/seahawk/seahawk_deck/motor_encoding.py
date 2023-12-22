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

import rclpy
from rclpy.node import Node 

# Import ros message types
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray

class Motor_encoding(Node):
    """
    Class that converts newtons to pwm values then to dshot packets
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__('motor_encoding')
        self.subscription = self.create_subscription(Float32MultiArray, 'kinematics', self._callback, 10)
        self.motor_pub = self.create_publisher(Int16MultiArray, 'motor_msgs', 10)
        
    @staticmethod
    def _convert_to_pwm(newtons:float):
        pass
    
    @staticmethod
    def _convert_to_dshot(pwm: int):
        pass

    @staticmethod
    def _create_packet(newton:float, telemetry: bool = False)->int:
        '''
        Generates a DSHOT packet given newtons and a telemetry request

        params:
            newton: A thrust value in newtons to be sent to a motor
            telemetry=False: Telemetry value (1 bit), True (1) if telemetry should be used, False (0) if not. Defaults to False
        returns:
            A 16 bit package of data to send following the pattern SSSSSSSSSSSTCCCC
        '''
        # Given newtons, convert to pwm valye
        pwm_throttle = Motor_encoding._convert_to_pwm(newton)
        
        # Given pwm value, convert to dshot
        dshot_throttle = Motor_encoding._convert_to_dshot(pwm_throttle)

        # Create DSHOT packet
        # Frames are organized in the following 16 bit pattern: SSSSSSSSSSSTCCCC
        # (S) 11 bit throttle
        # (T) 1 bit telemetry request
        # (C) 4 bit Cyclic Redundancy Check (CRC) (calculated in this function)
        # Shift everything over by one bit then append telemetry
        data = (dshot_throttle << 1) | telemetry
        # CRC calculation
        crc = (data ^ (data >> 4) ^ (data >> 8)) & 0x0F
        # Shift everything over by 4 bits then append crc
        return (data << 4) | crc

    def _callback(self, kine_msg):
        """Called every time kinematics publishes a message."""
        motor_msg = Int16MultiArray()
    
        # For every newton value provided by kinematics
        for index, newtons in enumerate(kine_msg().data):
            motor_msg.data[index] = Motor_encoding._create_packet(newtons)

        self.motor_pub.publish(motor_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Motor_encoding())
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
