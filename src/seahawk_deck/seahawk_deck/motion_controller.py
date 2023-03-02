
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

        # CONTROLLER KEYMAPPINGS
        controller = {
            'axes': ['left_stick', 'right_stick'],
            'left_stick': {
                'x':        -joy_msg.axes[0],
                'y':        joy_msg.axes[1],
                'press':    joy_msg.buttons[9],
            },
            'right_stick': {
                'x':        -joy_msg.axes[3],
                'y':        joy_msg.axes[4],
                'press':    joy_msg.buttons[10],
            },
            'left_trigger': joy_msg.axes[2],
            'right_trigger':joy_msg.axes[5],
            'dpad': {
                'up':       max(joy_msg.axes[7], 0), # +
                'down':     min(joy_msg.axes[7], 0), # -
                'right':    max(joy_msg.axes[6], 0), # +
                'left':     min(joy_msg.axes[6], 0), # -
            },
            'a':            joy_msg.buttons[0],
            'b':            joy_msg.buttons[1],
            'x':            joy_msg.buttons[2],
            'y':            joy_msg.buttons[3],
            'left_bumper':  joy_msg.buttons[4],
            'right_bumper': joy_msg.buttons[5],
            'window':       joy_msg.buttons[6],
            'menu':         joy_msg.buttons[7],
            'xbox':         joy_msg.buttons[8],
        }

        # NORMALIZE CONTROLLER AXES
        SCALE_ACCURACY = 10 * 2 # must be even
        for axis in controller['axes']:
            angle = math.atan(controller[axis]['x'] / (controller[axis]['y'] + 0.0000000000000001)) # don't want to divide by zero!
            scale = pow(pow(math.cos(angle), SCALE_ACCURACY) + pow(math.sin(angle), SCALE_ACCURACY), 1 / SCALE_ACCURACY) 
            
            controller[axis]['x'] *= scale
            controller[axis]['y'] *= scale

        # BINDINGS
        twist_msg = Twist()
        twist_msg.linear.x  = controller['left_stick']['y'] # X (forward)
        twist_msg.linear.y  = controller['left_stick']['x']# Y (right)
        twist_msg.linear.z  = (controller['left_trigger'] - controller['right_trigger']) / 2 # Z
        twist_msg.angular.x = 0.0 # R 
        twist_msg.angular.y = controller['right_stick']['y'] # P 
        twist_msg.angular.z = controller['right_stick']['x'] # Y 

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
        AXIS_SCALE = {
            'linear':  {'x': 1, 'y': 1, 'z': 1},    # forwards, sideways, depth
            'angular': {'x': 0, 'y': 1, 'z': 0.1},  # roll, pitch, yaw
        }


        # No roll, we do not want cartwheels 
        motor_msg.data[0] = twist_msg.linear.x - twist_msg.linear.y - twist_msg.angular.z
        # motor_msg.data[1] = twist_msg.linear.z + twist_msg.angular.y
        motor_msg.data[2] = -twist_msg.linear.x - twist_msg.linear.y + twist_msg.angular.z
        # motor_msg.data[3] = twist_msg.linear.z - twist_msg.angular.y
        motor_msg.data[4] = -twist_msg.linear.x + twist_msg.linear.y - twist_msg.angular.z
        # motor_msg.data[5] = twist_msg.linear.z - twist_msg.angular.y
        motor_msg.data[6] = twist_msg.linear.x + twist_msg.linear.y + twist_msg.angular.z
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
