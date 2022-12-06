
import sys 

import rclpy

from rclpy.node import Node 

from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import RelativeHumidity
from std_msgs.msg import Int16MultiArray

class ControlBridge(Node):
    """
    Class that implements the motion controller.
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__('control_bridge')

        # Publishers and pre-defined messages 

        self.imu0_msg = Imu() 
        self.imu1_msg = Imu() 
        self.temp_msg = Temperature()
        self.press_msg = FluidPressure()
        self.humid_msg = RelativeHumidity()

        ## TODO: Fix the frame_id of the IMU messages.
        self.imu0_msg.header.frame_id = 'base_link'
        self.imu1_msg.header.frame_id = 'base_link'
        self.temp_msg.header.frame_id = 'base_link'
        self.press_msg.header.frame_id = 'base_link'
        self.humid_msg.header.frame_id = 'base_link'
        
        ## TODO: Measure or estimate covariances. 
        self.imu0_msg.orientation_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1, 
        ]
        self.imu0_msg.angular_velocity_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1, 
        ]
        self.imu0_msg.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1, 
        ]
        self.imu1_msg.orientation_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1, 
        ]
        self.imu1_msg.angular_velocity_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1, 
        ]
        self.imu1_msg.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1, 
        ]

        ## TODO: Low priority: Measure the noise on these sensors.
        self.temp_msg.variance = 0.0 
        self.press_msg.variance = 0.0
        self.humid_msg.variance = 0.0 
        
        self.imu0_pub = self.create_publisher(Imu, 'imu0', 10)
        self.imu1_pub = self.create_publisher(Imu, 'imu1', 10)
        self.temp_pub = self.create_publisher(Temperature, 'temperature', 10)
        self.press_pub = self.create_publisher(FluidPressure, 'pressure', 10)
        self.humid_pub = self.create_publisher(RelativeHumidity, 'humidity', 10)

        # Subscribers 
        self.motor_sub = self.create_subscription(Int16MultiArray, 'drive/motors', self._motor_callback, 10)

        # Timer to publish data 
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self._timer_callback)

    def _timer_callback(self):
        """Publish sensor data"""

        #### TODO: Read all sensor data 

        ## Build IMU Messages
        ## http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html        
        ## http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Quaternion.html
        self.imu0_msg.orientation.w = 0.0
        self.imu0_msg.orientation.x = 0.0
        self.imu0_msg.orientation.y = 0.0
        self.imu0_msg.orientation.z = 0.0
        self.imu0_msg.angular_velocity.x = 0.0
        self.imu0_msg.angular_velocity.y = 0.0
        self.imu0_msg.angular_velocity.z = 0.0
        self.imu0_msg.linear_acceleration.x = 0.0
        self.imu0_msg.linear_acceleration.y = 0.0
        self.imu0_msg.linear_acceleration.z = 0.0

        self.imu1_msg.orientation.w = 0.0
        self.imu1_msg.orientation.x = 0.0
        self.imu1_msg.orientation.y = 0.0
        self.imu1_msg.orientation.z = 0.0
        self.imu1_msg.angular_velocity.x = 0.0
        self.imu1_msg.angular_velocity.y = 0.0
        self.imu1_msg.angular_velocity.z = 0.0
        self.imu1_msg.linear_acceleration.x = 0.0
        self.imu1_msg.linear_acceleration.y = 0.0
        self.imu1_msg.linear_acceleration.z = 0.0

        ## https://docs.ros2.org/foxy/api/sensor_msgs/msg/Temperature.html
        self.temp_msg.temperature = 0.0 

        ## https://docs.ros2.org/foxy/api/sensor_msgs/msg/FluidPressure.html
        self.press_msg.fluid_pressure = 0.0

        ## http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/RelativeHumidity.html
        self.humid_msg.relative_humidity = 0.0 

        self.imu0_pub.publish(self.imu0_msg)
        self.imu1_pub.publish(self.imu1_msg)
        self.temp_pub.publish(self.temp_msg)
        self.press_pub.publish(self.press_msg)
        self.humid_pub.publish(self.humid_msg)

    def _motor_callback(self, msg: Int16MultiArray):
        """Called when there's a motor control message."""
        
        m1 = msg.data[0]
        m2 = msg.data[1]
        m3 = msg.data[2]
        m4 = msg.data[3]
        m5 = msg.data[4]
        m6 = msg.data[5]
        m7 = msg.data[6]
        m8 = msg.data[7]

        ## TODO: Send motor commands. 

        self.get_logger().info(f"Driving motors: {m1} {m2} {m3} {m4} {m5} {m6} {m7} {m8}")


def main(args=None):    
    rclpy.init(args=args)
    rclpy.spin(ControlBridge())
    rclpy.shutdown()    


if __name__ == '__main__':
    main(sys.argv)
