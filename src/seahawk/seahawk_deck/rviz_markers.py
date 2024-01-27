import sys 
import math 
from enum import Enum

import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray

class MarkerMaker(Node):
    """
    Node to visualize the motor controls of the robot using RViz markers
    """
    # Position constants for top motors (0-3)
    X_TOP = 0.19
    Y_TOP = 0.12
    Z_TOP = 0.047
    # Position constants for top motors (4-7)
    X_BOT = 0.105
    Y_BOT = 0.15
    Z_BOT = -0.038

    # Motor positions
    # Position: (x, y, z)       , Rotation: (Roll, Pitch, Yaw)
    MOTORS = [
        ((-X_TOP,  Y_TOP, Z_TOP), (0,   math.pi / 2,  0)),      # 0 (Top)
        ((-X_TOP, -Y_TOP, Z_TOP), (0,   math.pi / 2,  0)),      # 1 (Top)
        (( X_TOP,  Y_TOP, Z_TOP), (0,   math.pi / 2,  0)),      # 2 (Top)
        (( X_TOP, -Y_TOP, Z_TOP), (0,   math.pi / 2,  0)),      # 3 (Top)
        (( X_BOT,  Y_BOT, Z_BOT), (0,   0,  3 * math.pi / 4)),  # 4 (Bottom)
        (( X_BOT, -Y_BOT, Z_BOT), (0,   0,  5 * math.pi / 4)),  # 5 (Bottom)    
        ((-X_BOT,  Y_BOT, Z_BOT), (0,   0,  math.pi     / 4)),  # 6 (Bottom)
        ((-X_BOT, -Y_BOT, Z_BOT), (0,   0,  7 * math.pi / 4)),  # 7 (Bottom)
    ]

    def __init__(self):
        """
        Initialize 'marker_maker' node
        """
        super().__init__("marker_maker")
        self.marker_pub = self.create_publisher(MarkerArray, "motor_debug", 10)
        self.subscription = self.create_subscription(Float32MultiArray, "motor_values", self.callback, 10)

        class ObjectTypes(Enum):
            ARROW, CUBE, TEXT_VIEW_FACING, = 0, 1, 9

        NUM_MOTORS = 8

        self.markers = MarkerArray()
        self.markers.markers = [ Marker() for x in range(NUM_MOTORS * 3) ]   
        self.boxes = self.markers.markers[0::3]
        self.labels = self.markers.markers[1::3]
        self.arrows = self.markers.markers[2::3]

        # Set IDs 
        for i, marker in enumerate(self.markers.markers):
            marker.header.frame_id = "base_link"
            marker.id = i 

        for i in range(NUM_MOTORS):
            self.labels[i].type = ObjectTypes.TEXT_VIEW_FACING.value
            self.labels[i].action = 0 # Add/Modify
            self.labels[i].scale.x = 0.05
            self.labels[i].scale.y = 0.05
            self.labels[i].scale.z = 0.05
            self.labels[i].pose.position.x = float(MarkerMaker.MOTORS[i][0][0]) + 0.5 * MarkerMaker.MOTORS[i][0][0] 
            self.labels[i].pose.position.y = float(MarkerMaker.MOTORS[i][0][1]) + 0.5 * MarkerMaker.MOTORS[i][0][1]
            self.labels[i].pose.position.z = float(MarkerMaker.MOTORS[i][0][2])
            self.labels[i].color.r = 1.0
            self.labels[i].color.g = 1.0
            self.labels[i].color.b = 1.0
            self.labels[i].color.a = 1.0
            self.labels[i].text = f"({i})0N"

            self.arrows[i].type = ObjectTypes.ARROW.value
            self.arrows[i].action = 0 # Add/Modify
            self.arrows[i].scale.x = 0.0
            self.arrows[i].scale.y = 0.05
            self.arrows[i].scale.z = 0.05
            self.arrows[i].pose.position.x = float(MarkerMaker.MOTORS[i][0][0])
            self.arrows[i].pose.position.y = float(MarkerMaker.MOTORS[i][0][1])
            self.arrows[i].pose.position.z = float(MarkerMaker.MOTORS[i][0][2])
            self.arrows[i].pose.orientation = self.quaternion_from_euler(*MarkerMaker.MOTORS[i][1])
            self.arrows[i].color.r = 1.0
            self.arrows[i].color.g = 0.0
            self.arrows[i].color.b = 0.0
            self.arrows[i].color.a = 1.0 # Full opacity

    def callback(self, motor_msg):
        """
        Called every time a message is published to 'motor_values'. Draws arrows and publishes markers to 'motor_debug'
        """
        MAX_FWD_THRUST = 36.3826715 / 4
        for i, motor in enumerate(motor_msg.data):
            self.arrows[i].scale.x = -motor / 16 # Scale arrows down to a reasonable size
            self.labels[i].text = f"({i}){round(motor, 2)}N"
            self.arrows[i].color.r = abs(motor) / MAX_FWD_THRUST
            self.arrows[i].color.g = 0.2
            self.arrows[i].color.b = 0.45

        self.marker_pub.publish(self.markers)

    @staticmethod
    def quaternion_from_euler(ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci * ck
        cs = ci * sk
        sc = si * ck
        ss = si * sk
    
        q = Quaternion()
        q.x = cj * sc - sj * cs
        q.y = cj * ss + sj * cc
        q.z = cj * cs - sj * sc
        q.w = cj * cc + sj * ss

        return q


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MarkerMaker())
    rclpy.shutdown()    


if __name__ == "__main__":
    main(sys.argv)