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

    MOTOR_X = 0.22
    MOTOR_Y = 0.22
    TOP_Z   = 0.10
    BOT_Z   = -0.05
    TOP_P   = 47 * math.pi / 36

    # Motor positions
    # (x, y, z spatial position)    , (rotation)
    MOTORS = [
        (( MOTOR_X,  MOTOR_Y, TOP_Z), (0,   TOP_P,  math.pi     / 4)),
        (( MOTOR_X, -MOTOR_Y, TOP_Z), (0,   TOP_P,  7 * math.pi / 4)),
        ((-MOTOR_X,  MOTOR_Y, TOP_Z), (0,   TOP_P,  3 * math.pi / 4)),
        ((-MOTOR_X, -MOTOR_Y, TOP_Z), (0,   TOP_P,  5 * math.pi / 4)),
        (( MOTOR_X,  MOTOR_Y, BOT_Z), (0,   0,      3 * math.pi / 4)),
        (( MOTOR_X, -MOTOR_Y, BOT_Z), (0,   0,      5 * math.pi / 4)),     
        ((-MOTOR_X,  MOTOR_Y, BOT_Z), (0,   0,      math.pi     / 4)),
        ((-MOTOR_X, -MOTOR_Y, BOT_Z), (0,   0,      7 * math.pi / 4)),
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
            self.labels[i].text = f"({i})0N"

            # self.boxes[i].type = ObjectTypes.CUBE.value # Cube 
            # self.boxes[i].action = 0 # Add/Modify
            # self.boxes[i].scale.x = 0.05
            # self.boxes[i].scale.y = 0.05
            # self.boxes[i].scale.z = 0.05
            # self.boxes[i].pose.position.x = float(MarkerMaker.MOTORS[i][0][0])
            # self.boxes[i].pose.position.y = float(MarkerMaker.MOTORS[i][0][1])
            # self.boxes[i].pose.position.z = float(MarkerMaker.MOTORS[i][0][2])
            # self.boxes[i].pose.orientation = self._quaternion_from_euler(*MarkerMaker.MOTORS[i][1])
            # self.boxes[i].color.r = 1.0
            # self.boxes[i].color.g = 1.0
            # self.boxes[i].color.b = 1.0
            # self.boxes[i].color.a = 0.5

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
        for i, motor in enumerate(motor_msg.data):
            self.arrows[i].scale.x = -motor / 4
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



# TODO list
# 1. Fix this error (done)
    # ❯ ros2 launch seahawk kinematics_viz.launch.py
    # [INFO] [launch]: All log files can be found below /home/steph/.ros/log/2024-01-24-02-38-43-893948-steph-ThinkPad-L14-Gen-3-155587
    # [INFO] [launch]: Default logging verbosity is set to INFO
    # [ERROR] [launch]: Caught exception in launch (see debug for traceback): "package 'joint_state_publisher' not found, searching: ['/home/steph/Documents/robotics/mate_2024/cabrillo_rov_2023/install/seahawk_msgs', '/home/steph/Documents/robotics/mate_2024/cabrillo_rov_2023/install/seahawk_description', '/home/steph/Documents/robotics/mate_2024/cabrillo_rov_2023/install/seahawk', '/home/steph/Documents/robotics/mate_2024/cabrillo_rov_2023/install/h264_image_transport', '/home/steph/Documents/robotics/mate_2024/cabrillo_rov_2023/install/h264_msgs', '/opt/ros/humble']"
# 2. Look at launch file
# 3. See if rviz file is needed
# 4. Understand quaternion_from_euler()
# 5. Add gradients to thrust vectors
# 6. Scale thrust vectors down
# 7. Integrate new files with rviz
