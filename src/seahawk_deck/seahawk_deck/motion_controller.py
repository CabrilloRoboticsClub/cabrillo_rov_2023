import sys 

import math

import rclpy

from rclpy.node import Node 

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class MotionController(Node):
    """
    Class that implements the motion controller.
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__('motion_controller')
        self.subscription = self.create_subscription(Twist, 'drive/twist', self._callback, 10)
        self.motor_pub = self.create_publisher(Float32MultiArray, 'drive/motors', 10)
    
    def thrust_control(self, direction1:float, direction2:float)->float:
        """Add two directions in such a way that they do not fall outside [-1, 1]"""
        if (direction1 >= 0 and direction2 >= 0):
            return direction1 + direction2 - (direction1 * direction2)
        elif (direction1 < 0 and direction2 < 0):
            return direction1 + direction2 + (direction1 * direction2)
        else:
            return direction1 + direction2

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
        # 6/^ ^\0
        # 4\, ,/2
        # Odd are on top; thrust forward is down
        # 35° angle(7π/36)
        # ^FRONT^
        #  7   1
        #  5   3

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

        # Lower motors 
        motor_msg.data[0] = self.thrust_control(self.thrust_control(twist_msg.linear.x, -twist_msg.linear.y), -twist_msg.angular.z)
        motor_msg.data[2] = self.thrust_control(self.thrust_control(-twist_msg.linear.x, -twist_msg.linear.y), twist_msg.angular.z)
        motor_msg.data[4] = self.thrust_control(self.thrust_control(-twist_msg.linear.x, twist_msg.linear.y), -twist_msg.angular.z)
        motor_msg.data[6] = self.thrust_control(self.thrust_control(twist_msg.linear.x, twist_msg.linear.y), twist_msg.angular.z)

        # Upper motors
        motor_msg.data[1] = self.thrust_control(twist_msg.linear.z, twist_msg.angular.y)
        motor_msg.data[3] = self.thrust_control(twist_msg.linear.z, -twist_msg.angular.y)
        motor_msg.data[5] = self.thrust_control(twist_msg.linear.z, -twist_msg.angular.y)
        motor_msg.data[7] = self.thrust_control(twist_msg.linear.z, twist_msg.angular.y)
        

        # Publish data to the motors
        self.motor_pub.publish(motor_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MotionController())
    rclpy.shutdown()    


if __name__ == '__main__':
    main(sys.argv)
