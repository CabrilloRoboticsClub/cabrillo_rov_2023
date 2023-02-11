
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
        self.motor_pub = self.create_publisher(Int16MultiArray, 'drive/motors', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self._callback, 10)

    def _callback(self, joy_msg):
        """
        Called every time the joystick publishes a message. 
        """
        self.get_logger().info(f"Joystick axes: {joy_msg.axes} buttons: {joy_msg.buttons}")

        # Compute desired motion in <x, y, z, r, p, y>
        # Message index 3 and 4 are the right stick left/right and forward/back
        # Yaw is at position 2 and 5. Domain is [-1, 1]

        """
        twist_msg = Twist()
        twist_msg.linear.x  = joy_msg.axes[4] # X 
        twist_msg.linear.y  = -joy_msg.axes[3]# Y Direction was inverted. Negative added so negative is to the left and positive is to the right
        twist_msg.linear.z  = joy_msg.axes[1] # Z 
        twist_msg.angular.x = 0.0 # R 
        twist_msg.angular.y = 0.0 # P 
        twist_msg.angular.z = (joy_msg.axes[2] - joy_msg.axes[5]) / 2 # Y 
        """

        twist_msg = Twist()
        twist_msg.linear.x  = (joy_msg.axes[2] - joy_msg.axes[5]) / 2 # X 
        twist_msg.linear.y  = -joy_msg.axes[0] # Y Direction was inverted. Negative added so negative is to the left and positive is to the right
        twist_msg.linear.z  = joy_msg.axes[1] # Z 
        twist_msg.angular.x = 0.0 # R 
        twist_msg.angular.y = joy_msg.axes[4] # P 
        twist_msg.angular.z = -joy_msg.axes[3] # Y 

        # Send the twist message for debugging.
        self.twist_pub.publish(twist_msg)

        # Convert the X,Y,Z,R,P,Y to thrust settings for each motor. 
        motor_msg = Int16MultiArray()
        # +1 = Full thrust, Forwards
        #  0 = Off
        # -1 = Full thrust, Backwards
        # Even are on bottom
        # ^FRONT^
        # 6/^ ^\0
        # 4\, ,/2
        # Odd are on top; thrust forward is down
        # ^FRONT^
        #  7   1
        #  5   3
        motor_msg.data = [
            0,  # Motor 0 thrust 
            0,  # Motor 1 thrust
            0,  # Motor 2 thrust
            0,  # Motor 3 thrust
            0,  # Motor 4 thrust
            0,  # Motor 5 thrust
            0,  # Motor 6 thrust
            0,  # Motor 7 thrust
        ]

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
