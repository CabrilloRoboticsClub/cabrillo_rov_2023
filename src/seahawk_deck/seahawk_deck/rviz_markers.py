
import rclpy
from rclpy.node import Node 

from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Joy 
from std_msgs.msg import Float32MultiArray, ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray

import sys 
import math 


class MarkerMaker(Node):
    """
    Visualize the motor controls of the robot using RViz markers
    """

    MOTORS = [
        ((1,1,0),       (0,0,7*math.pi/4)),  
        ((1,1,0.5),     (0,7*math.pi/36,math.pi/4)),
        ((1,-1,0),      (0,0,math.pi/4)),
        ((1,-1,0.5),    (0,7*math.pi/36,7*math.pi/4)),
        ((-1,-1,0),     (0,0,3*math.pi/4)),
        ((-1,-1,0.5),   (0,7*math.pi/36,5*math.pi/4)),
        ((-1,1,0),      (0,0,5*math.pi/4)),
        ((-1,1,0.5),    (0,7*math.pi/36,3*math.pi/4)),
    ]

    def __init__(self):
        """Initialize this node"""
        super().__init__('marker_maker')
        self.marker_pub = self.create_publisher(MarkerArray, 'drive/motors_debug', 10)
        self.subscription = self.create_subscription(Float32MultiArray, 'drive/motors', self._callback, 10)
        self.labels = MarkerArray()
        self.labels.markers = [
            Marker(),
            Marker(),
            Marker(),
            Marker(),
            Marker(),
            Marker(),
            Marker(),
            Marker(),
        ]
        self.boxes = MarkerArray()
        self.boxes.markers = [
            Marker(),
            Marker(),
            Marker(),
            Marker(),
            Marker(),
            Marker(),
            Marker(),
            Marker(),
        ]
        self.markers = MarkerArray()
        self.markers.markers = [
            Marker(),
            Marker(),
            Marker(),
            Marker(),
            Marker(),
            Marker(),
            Marker(),
            Marker(),
        ]

        # Show motor labels
        for id, marker in enumerate(self.labels.markers):
            marker.header.frame_id = 'map' # Broken but easy
            marker.id = id + 20
            marker.type = 9 # Text 
            marker.action = 0 # Add/Modify
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.pose.position.x = float(MarkerMaker.MOTORS[id][0][0])
            marker.pose.position.y = float(MarkerMaker.MOTORS[id][0][1])
            marker.pose.position.z = float(MarkerMaker.MOTORS[id][0][2])
            #marker.pose.orientation = self._quaternion_from_euler(*MarkerMaker.MOTORS[id][1])
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.text = str(id)

        # Show motor positions
        for id, marker in enumerate(self.boxes.markers):
            marker.header.frame_id = 'map' # Broken but easy
            marker.id = id + 10
            marker.type = 1 # Cube 
            marker.action = 0 # Add/Modify
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.pose.position.x = float(MarkerMaker.MOTORS[id][0][0])
            marker.pose.position.y = float(MarkerMaker.MOTORS[id][0][1])
            marker.pose.position.z = float(MarkerMaker.MOTORS[id][0][2])
            marker.pose.orientation = self._quaternion_from_euler(*MarkerMaker.MOTORS[id][1])
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.5

        for id, marker in enumerate(self.markers.markers):
            marker.header.frame_id = 'map' # Broken but easy
            marker.id = id
            marker.type = 0 # Arrow 
            marker.action = 0 # Add/Modify
            marker.scale.x = 0.0
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.pose.position.x = float(MarkerMaker.MOTORS[id][0][0])
            marker.pose.position.y = float(MarkerMaker.MOTORS[id][0][1])
            marker.pose.position.z = float(MarkerMaker.MOTORS[id][0][2])
            marker.pose.orientation = self._quaternion_from_euler(*MarkerMaker.MOTORS[id][1])

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
 
    def _callback(self, motor_msg):
        """
        When I see ne new message on drive/motors
        """
        self.get_logger().info(f"Motor Message: {motor_msg.data}")
        for i, motor in enumerate(motor_msg.data):
            self.markers.markers[i].scale.x = motor
            self.labels.markers[i].text = f"{i}: {motor}"
            if motor < -1 or motor > 1:
                self.markers.markers[i].color.r = 1.0
                self.markers.markers[i].color.g = 0.0
                self.markers.markers[i].color.b = 0.0
                self.labels.markers[i].color.r = 1.0
                self.labels.markers[i].color.g = 0.0
                self.labels.markers[i].color.b = 0.0
            else:
                self.markers.markers[i].color.r = 0.0
                self.markers.markers[i].color.g = 1.0
                self.markers.markers[i].color.b = 1.0
                self.labels.markers[i].color.r = 1.0
                self.labels.markers[i].color.g = 1.0
                self.labels.markers[i].color.b = 1.0

        self.marker_pub.publish(self.boxes)
        self.marker_pub.publish(self.labels)
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
