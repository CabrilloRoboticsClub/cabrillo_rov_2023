import sys 

import rclpy

from rclpy.node import Node 

from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Joy

class Input(Node):
    """
    Class that implements the motion controller.
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__('input_xbox_one')
        self.subscription = self.create_subscription(Joy, 'joy', self._callback, 10)
        self.twist_pub = self.create_publisher(Twist, 'drive/twist', 10)

    def _callback(self, joy_msg):
        """Called every time the joystick publishes a message."""
        self.get_logger().info(f"Joystick axes: {joy_msg.axes} buttons: {joy_msg.buttons}")

        # Compute desired motion in <x, y, z, r, p, y>

        # CONTROLLER KEYMAPPINGS
        # for more controller mappings see commit 4b3ba9b6f51c15a86bdab61bcc5f012b8b3dbd06 motion_controller.py line 41
        # https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023/blob/4b3ba9b6f51c15a86bdab61bcc5f012b8b3dbd06/src/seahawk_deck/seahawk_deck/motion_controller.py#LL41
        controller = {
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
        } 

        # BINDINGS
        twist_msg = Twist()
        twist_msg.linear.x  = controller['left_stick']['y'] # X (forwards)
        twist_msg.linear.y  = controller['left_stick']['x']# Y (sideways)
        twist_msg.linear.z  = (controller['left_trigger'] - controller['right_trigger']) / 2 # Z (depth)
        twist_msg.angular.x = 0.0 # R (roll) (we don't need roll)
        twist_msg.angular.y = controller['right_stick']['y'] # P (pitch) 
        twist_msg.angular.z = controller['right_stick']['x'] # Y (yaw)

        self.twist_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Input())
    rclpy.shutdown()    


if __name__ == '__main__':
    main(sys.argv)