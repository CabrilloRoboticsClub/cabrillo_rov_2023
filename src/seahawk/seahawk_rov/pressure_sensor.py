import rclpy
from rclpy import Node

class PressureReading (Node):
    def __init__(self):
        super().__init__('pressure_publisher')  # what?
        self.publisher_ = self.create_publisher(String, 'pressure_topic', 10)  # create a publisher that publishes messages of type String to pressure_topic
        timer_period = 0.5  # space messages out by 0.5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)  # create a timer for when the messages are published to topic

    def timer_callback(self):
        msg = String()  # create a obj of type String
        msg.data = "hamburger"  # pressure data will go here later
        self.publisher_.publish(msg)  # publish msg to pressure_topic


def main(args=None):
    rclpy.init(args=args)  # what is this
    pressure_obj = PressureReading()  # create object publisher of type PressureReading
    rclpy.spin(pressure_obj)  # keep the publisher object running

    pressure_obj.destroy_node()  # terminate program when not in use
    rclpy.shutdown()  # shut down the node

if __name__ == '__main__':  # what is this
    main()  # call main?

