
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Sub(Node):

    def __init__(self):
        super().__init__("Subby")
        self.subscription = self.create_subscription (
            String,
            "sample_topic",
            self.receive,
            10
        )
        self.subscription2 = self.create_subscription (
            String,
            "secondary_topic",
            self.receive,
            10
        )


    def receive(self, msg):
        self.get_logger().info("I heard a message! : '%s'" % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minSub = Sub()
    rclpy.spin(minSub)

    minSub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
