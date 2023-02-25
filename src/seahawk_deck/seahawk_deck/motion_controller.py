
import sys 

import math

import rclpy

from rclpy.node import Node 

from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Joy 
from std_msgs.msg import Float32MultiArray

class MotionController(Node):
    """
    Class that implements the motion controller.
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__('motion_controller')
        self.twist_pub = self.create_publisher(Twist, 'drive/twist', 10)
        self.motor_pub = self.create_publisher(Float32MultiArray, 'drive/motors', 10)
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

        joy_lin_x = joy_msg.axes[1] # left y
        joy_lin_y = -joy_msg.axes[0] # left x
        joy_lin_z = (joy_msg.axes[2] - joy_msg.axes[5]) / 2 # Triggers
        joy_ang_x = 0.0 # no roll
        joy_ang_y = joy_msg.axes[4] # right y
        joy_ang_z = joy_msg.axes[3] # right x

        if joy_lin_x > joy_lin_y:
            scale = math.sqrt(pow(joy_lin_y / joy_lin_x - joy_lin_y, 2) + pow(1 - joy_lin_x, 2))
        elif joy_lin_y > joy_lin_x:
            scale = math.sqrt(pow(joy_lin_x / joy_lin_y - joy_lin_x, 2) + pow(1 - joy_lin_y, 2))
        elif joy_lin_x == joy_lin_y:
            scale = math.sqrt(2) - 1

        twist_msg = Twist()
        twist_msg.linear.x  = joy_lin_x # X 
        twist_msg.linear.y  = joy_lin_y # Y Direction was inverted. Negative added so negative is to the left and positive is to the right
        twist_msg.linear.z  = joy_lin_z # Z
        twist_msg.angular.x = joy_ang_x # R 
        twist_msg.angular.y = joy_ang_y # P 
        twist_msg.angular.z = joy_ang_z # Y 

        # Send the twist message for debugging.
        self.twist_pub.publish(twist_msg)

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
        
        # No roll, we do not want cartwheels 
        motor_msg.data[0] = 1 / scale * (twist_msg.linear.x - twist_msg.linear.y) + twist_msg.angular.z
        # motor_msg.data[1] = twist_msg.linear.z + twist_msg.angular.y
        motor_msg.data[2] = 1 / scale * (-twist_msg.linear.x - twist_msg.linear.y) - twist_msg.angular.z
        # motor_msg.data[3] = twist_msg.linear.z - twist_msg.angular.y
        motor_msg.data[4] = 1 / scale * (-twist_msg.linear.x + twist_msg.linear.y) - twist_msg.angular.z
        # motor_msg.data[5] = twist_msg.linear.z - twist_msg.angular.y
        motor_msg.data[6] = 1 / scale * (twist_msg.linear.x + twist_msg.linear.y) + twist_msg.angular.z
        # motor_msg.data[7] = twist_msg.linear.z + twist_msg.angular.y

        # Publish data to the motors
        self.motor_pub.publish(motor_msg)

"""
def linear_x(input_data):
    # Forward and Backward
    #   LX+:             LX-:
    # [0+ 2-]          [0- 2+]
    # [4- 6+]          [4+ 6-]

def linear_y(input_data):
    # Side to Side
    #   LY+:             LY-:
    # [0- 2-]          [0+ 2+]
    # [4+ 6+]          [4- 6-]

def linear_z(input_data):
    # Dive and Surface
    #   LZ+:             LZ-
    # [1+ 3+]          [1- 3-]
    # [5+ 7+]          [5- 7-]

def angular_y(input_data):
    # Pitch (aim up/down)
    #   AY+:             AY-:
    # [1+ 3-]          [1- 3+]
    # [5- 7+]          [5+ 7-]

def angular_z(input_data):
    # Yaw (turning)
    #   AZ+:             AZ-:
    # [0- 2+]          [0+ 2-]
    # [4- 6+]          [4+ 6-]

"""

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MotionController())
    rclpy.shutdown()    


if __name__ == '__main__':
    main(sys.argv)
