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
    
    def thrust_control(self, direction1:float, direction2:float)->float:
        """Add two directions in such a way that they do not fall outside [-1, 1]"""
        if (direction1 >= 0 and direction2 >= 0):
            return direction1 + direction2 - (direction1 * direction2)
        elif (direction1 < 0 and direction2 < 0):
            return direction1 + direction2 + (direction1 * direction2)
        else:
            return direction1 + direction2

    def _callback(self, joy_msg):
        """Called every time the joystick publishes a message. """
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

        # BINDINGS
        twist_msg = Twist()
        twist_msg.linear.x  = controller['left_stick']['y'] # X (forwards)
        twist_msg.linear.y  = controller['left_stick']['x']# Y (sideways)
        twist_msg.linear.z  = (controller['left_trigger'] - controller['right_trigger']) / 2 # Z (depth)
        twist_msg.angular.x = 0.0 # R (roll)
        twist_msg.angular.y = controller['right_stick']['y'] # P (pitch) 
        twist_msg.angular.z = controller['right_stick']['x'] # Y (yaw)

        # CONSTANT AXIS SCALING
        AXIS_SCALE = {
            'linear':  {'x': 1, 'y': 1, 'z': 1},    # forwards, sideways, depth
            'angular': {'x': 0, 'y': 0.5, 'z': 0.5},  # roll, pitch, yaw
        }
        twist_msg.linear.y  *= AXIS_SCALE['linear']['x']
        twist_msg.linear.x  *= AXIS_SCALE['linear']['y']
        twist_msg.linear.z  *= AXIS_SCALE['linear']['z']
        twist_msg.angular.x *= AXIS_SCALE['angular']['x']
        twist_msg.angular.y *= AXIS_SCALE['angular']['y']
        twist_msg.angular.z *= AXIS_SCALE['angular']['z']

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
