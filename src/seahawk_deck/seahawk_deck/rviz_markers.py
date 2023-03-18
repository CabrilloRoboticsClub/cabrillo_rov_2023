
import rclpy
from rclpy.node import Node 

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray

import sys 
import math 

class MarkerMaker(Node):
    """
    Visualize the motor controls of the robot using RViz markers
    """

    MOTOR_X = 0.22
    MOTOR_Y = 0.22
    TOP_Z = 0.10
    BOT_Z = -0.05
    TOP_P = 47*math.pi/36

    MOTORS = [
        (( MOTOR_X, -MOTOR_Y, BOT_Z), (0,0,5*math.pi/4)),  
        (( MOTOR_X, -MOTOR_Y, TOP_Z), (0,TOP_P,7*math.pi/4)),
        ((-MOTOR_X, -MOTOR_Y, BOT_Z), (0,0,7*math.pi/4)),
        ((-MOTOR_X, -MOTOR_Y, TOP_Z), (0,TOP_P,5*math.pi/4)),
        ((-MOTOR_X,  MOTOR_Y, BOT_Z), (0,0,math.pi/4)),
        ((-MOTOR_X,  MOTOR_Y, TOP_Z), (0,TOP_P,3*math.pi/4)),
        (( MOTOR_X,  MOTOR_Y, BOT_Z), (0,0,3*math.pi/4)),
        (( MOTOR_X,  MOTOR_Y, TOP_Z), (0,TOP_P,math.pi/4)),
    ]

    def __init__(self):
        """Initialize this node"""
        super().__init__('marker_maker')
        self.marker_pub = self.create_publisher(MarkerArray, 'drive/motors_debug', 10)
        self.subscription = self.create_subscription(Float32MultiArray, 'drive/motors', self._callback, 10)

        self.markers = MarkerArray()
        self.markers.markers = [ Marker() for x in range(8*3) ]   
        self.boxes = self.markers.markers[0::3]
        self.labels = self.markers.markers[1::3]
        self.arrows = self.markers.markers[2::3]

        # Set IDs 
        for i, marker in enumerate(self.markers.markers):
            marker.header.frame_id = 'base_link' 
            marker.id = i 

        for i in range(8):
            self.labels[i].type = 9 # Text 
            self.labels[i].action = 0 # Add/Modify
            self.labels[i].scale.x = 0.1
            self.labels[i].scale.y = 0.1
            self.labels[i].scale.z = 0.1
            self.labels[i].pose.position.x = float(MarkerMaker.MOTORS[i][0][0]) + 0.5 * MarkerMaker.MOTORS[i][0][0] 
            self.labels[i].pose.position.y = float(MarkerMaker.MOTORS[i][0][1]) + 0.5 * MarkerMaker.MOTORS[i][0][1]
            self.labels[i].pose.position.z = float(MarkerMaker.MOTORS[i][0][2])
            self.labels[i].color.r = 1.0
            self.labels[i].color.g = 1.0
            self.labels[i].color.b = 1.0
            self.labels[i].color.a = 1.0
            self.labels[i].text = str(i)

            self.boxes[i].type = 1 # Cube 
            self.boxes[i].action = 0 # Add/Modify
            self.boxes[i].scale.x = 0.05
            self.boxes[i].scale.y = 0.05
            self.boxes[i].scale.z = 0.05
            self.boxes[i].pose.position.x = float(MarkerMaker.MOTORS[i][0][0])
            self.boxes[i].pose.position.y = float(MarkerMaker.MOTORS[i][0][1])
            self.boxes[i].pose.position.z = float(MarkerMaker.MOTORS[i][0][2])
            self.boxes[i].pose.orientation = self._quaternion_from_euler(*MarkerMaker.MOTORS[i][1])
            self.boxes[i].color.r = 1.0
            self.boxes[i].color.g = 1.0
            self.boxes[i].color.b = 1.0
            self.boxes[i].color.a = 0.5

            self.arrows[i].type = 0 # Arrow 
            self.arrows[i].action = 0 # Add/Modify
            self.arrows[i].scale.x = 0.0
            self.arrows[i].scale.y = 0.05
            self.arrows[i].scale.z = 0.05
            self.arrows[i].pose.position.x = float(MarkerMaker.MOTORS[i][0][0])
            self.arrows[i].pose.position.y = float(MarkerMaker.MOTORS[i][0][1])
            self.arrows[i].pose.position.z = float(MarkerMaker.MOTORS[i][0][2])
            self.arrows[i].pose.orientation = self._quaternion_from_euler(*MarkerMaker.MOTORS[i][1])
            self.arrows[i].color.r = 1.0
            self.arrows[i].color.g = 0.0
            self.arrows[i].color.b = 0.0
            self.arrows[i].color.a = 1.0


    def _callback(self, motor_msg):
        """
        When I see new message on drive/motors
        """
        self.get_logger().info(f"Motor Message: {motor_msg.data}")
        for i, motor in enumerate(motor_msg.data):
            self.arrows[i].scale.x = motor
            self.labels[i].text = f"{i}:{round(motor,2)}"
            if motor < -1 or motor > 1:
                self.arrows[i].color.r = 1.0
                self.arrows[i].color.g = 0.0
                self.arrows[i].color.b = 0.0
                self.labels[i].color.r = 1.0
                self.labels[i].color.g = 0.0
                self.labels[i].color.b = 0.0
            else:
                self.arrows[i].color.r = 0.0
                self.arrows[i].color.g = 1.0
                self.arrows[i].color.b = 1.0
                self.labels[i].color.r = 1.0
                self.labels[i].color.g = 1.0
                self.labels[i].color.b = 1.0

        self.marker_pub.publish(self.markers)

    @staticmethod
    def _quaternion_from_euler(ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk
    
        q = Quaternion()
        q.x = cj*sc - sj*cs
        q.y = cj*ss + sj*cc
        q.z = cj*cs - sj*sc
        q.w = cj*cc + sj*ss

        return q


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MarkerMaker())
    rclpy.shutdown()    


if __name__ == '__main__':
    main(sys.argv)
