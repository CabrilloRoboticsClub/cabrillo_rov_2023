
import sys 

import rclpy

from rclpy.node import Node 

from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Joy 
from std_msgs.msg import Int16MultiArray

class MotionController(Node):
    """
    Class that implements the motion controller.
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__('motion_controller')
        self.twist_pub = self.create_publisher(Twist, 'drive/twist', 10)
        self.motor_pub = self.create_publisher(Int16MultiArray, 'drive/motors', 10) # subscription to this in rov/i2c_proxy
        self.subscription = self.create_subscription(Joy, 'joy', self._callback, 10)

    def _callback(self, joy_msg):
        """
        Called every time the joystick publishes a message. 
        """
        self.get_logger().info(f"Joystick axes: {joy_msg.axes} buttons: {joy_msg.buttons}")

        # Compute desired motion in <x, y, z, r, p, y>
        twist_msg = Twist()
        twist_msg.linear.x  = 0.0 # X 
        twist_msg.linear.y  = 0.0 # Y 
        twist_msg.linear.z  = 0.0 # Z 
        twist_msg.angular.x = 0.0 # R 
        twist_msg.angular.y = 0.0 # P 
        twist_msg.angular.z = 0.0 # Y 

        # Send the twist message for debugging.
        self.twist_pub.publish(twist_msg)

        # Convert the X,Y,Z,R,P,Y to thrust settings for each motor. 
        motor_msg = Int16MultiArray()
        motor_msg.data = [
            0,  # Motor 1 thrust 
            0,  # Motor 2 thrust
            0,  # Motor 3 thrust
            0,  # Motor 4 thrust
            0,  # Motor 5 thrust
            0,  # Motor 6 thrust
            0,  # Motor 7 thrust
            0,  # Motor 8 thrust
        ]
        # int16. A 16-bit signed integer whose values exist on the interval [−32,767, +32,767] .

        # Validate motor power. Limit the sum of power to groups of motors. 
        motor_msg.data[0] = 1 

        # Publish data to the motors
        self.motor_pub.publish(motor_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MotionController())
    rclpy.shutdown()    


if __name__ == '__main__':
    main(sys.argv)
