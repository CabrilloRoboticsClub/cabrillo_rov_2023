import sys 

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
    
    # Calculates what a thruster should output based on multiple input values
    def combine_input(self, direction1:float, direction2:float)->float:
        """Add two directions in such a way that they do not fall outside [-1, 1]"""
        if (direction1 >= 0 and direction2 >= 0): # If both input values are positive (0 included)
            # Combines decimal percentage values based on a probability union operation to determine what the thruster should output
            return direction1 + direction2 - (direction1 * direction2)
        elif (direction1 < 0 and direction2 < 0): # If both input values are negative
            # Tweaked probability union operation similar to above to work with negative values
            return direction1 + direction2 + (direction1 * direction2)
        # If one value is positive and one value is negative, adds the values of different signs to offset each other
        return direction1 + direction2

    def _callback(self, joy_msg):
        """Called every time the joystick publishes a message. """
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
        twist_msg.angular.x = 0.0 # R (roll)
        twist_msg.angular.y = controller['right_stick']['y'] # P (pitch) 
        twist_msg.angular.z = controller['right_stick']['x'] # Y (yaw)

        # CONSTANT AXIS SCALING
        AXIS_SCALE = {
            'linear':  {'x': 1, 'y': 1, 'z': 1},    # forwards, sideways, depth
            'angular': {'x': 0, 'y': 0.5, 'z': 0.5},  # roll, pitch, yaw
        }
        twist_msg.linear.x *= AXIS_SCALE['linear']['x']
        twist_msg.linear.y  *= AXIS_SCALE['linear']['y']
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
        # int16. A 16-bit signed integer whose values exist on the interval [−32,767, +32,767] .

        # Lower motors 
        motor_msg.data[0] = self.combine_input(self.combine_input(twist_msg.linear.x, -twist_msg.linear.y), -twist_msg.angular.z)
        motor_msg.data[2] = self.combine_input(self.combine_input(-twist_msg.linear.x, -twist_msg.linear.y), twist_msg.angular.z)
        motor_msg.data[4] = self.combine_input(self.combine_input(-twist_msg.linear.x, twist_msg.linear.y), -twist_msg.angular.z)
        motor_msg.data[6] = self.combine_input(self.combine_input(twist_msg.linear.x, twist_msg.linear.y), twist_msg.angular.z)

        # Upper motors
        motor_msg.data[1] = self.combine_input(twist_msg.linear.z, twist_msg.angular.y)
        motor_msg.data[3] = self.combine_input(twist_msg.linear.z, -twist_msg.angular.y)
        motor_msg.data[5] = self.combine_input(twist_msg.linear.z, -twist_msg.angular.y)
        motor_msg.data[7] = self.combine_input(twist_msg.linear.z, twist_msg.angular.y)
        

        # Publish data to the motors
        self.motor_pub.publish(motor_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MotionController())
    rclpy.shutdown()    


if __name__ == '__main__':
    main(sys.argv)
