'''
publisher.py

Sample publisher feeding nothing but nonsense to anyone that listens.

Copyright (C) 2022 Cabrillo Robotics Club

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

class Pub(Node):

    def __init__(self):
        super().__init__("DummyPublisher")
        self.publisher = self.create_publisher(String, "sample_topic", 10)
        self.publisher2 = self.create_publisher(String, "secondary_topic", 10)
        self.timer = self.create_timer(1, self.emit_message)
        self.timer = self.create_timer(2, self.emit_secondary)
        self.i = 0

    def emit_message(self):
        mesg = String() # gotta use a ROS String, not Pythong str()
        mesg.data = "Hey, what's up scrub. Check out this counter %d" % self.i
        self.publisher.publish(mesg)
        self.get_logger().info("publishing: '%s'" % mesg.data)
        self.i += 1

    def emit_secondary(self):
        mesg = String()
        mesg.data = "secondary message %d" % self.i
        self.publisher2.publish(mesg)
        self.get_logger().info("secondary emit: '%s'" % mesg.data)

def main(args = None):
    rclpy.init(args=args)
    minPub = Pub()
    rclpy.spin(minPub)

    minPub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
