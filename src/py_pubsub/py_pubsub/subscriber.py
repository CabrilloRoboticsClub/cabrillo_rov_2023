'''
subscriber.py

Subscribes to nonsense; ingesting it, doing very little processing, and then
regurgitating it for anyone around to hear as well. Just like your weird uncle.

Copyright (C) 2022-2023 Cabrillo Robotics Club

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

Cabrillo Robotics Club
6500 Soquel Drive Aptos, CA 95003
cabrillorobotics@gmail.com
'''

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



    def receive(self, msg):
        self.get_logger().info("I heard a message! : '%s'" % msg.data)

class Dom(Node):
    def __init__(self):
        super().__init__("Dommy")
        self.subscription2 = self.create_subscription (
            String,
            "secondary_topic",
            self.receive,
            10
        )
    
    def receive(self, msg):
        self.get_logger().info("GOTTA DIFFERENTIATE SOMEHOW! : '%s'" % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minSub = Sub()
    maxDom = Dom()
    while(True):
        rclpy.spin_once(minSub)
        rclpy.spin_once(maxDom)

    minSub.destroy_node()
    maxDom.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
