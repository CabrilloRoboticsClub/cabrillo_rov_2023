#!/usr/bin/env python3
import rclpy 
import psutil
from rclpy.node import Node 
from std_msgs.msg import String


class DebugNode(Node):
    def __init__(self):
        super().__init__("debug")
        self._publisher = self.create_publisher(String, "debug_info", 10)
        self.timer = self.create_timer(0.5, self.pub_callback) 
    
    def pub_callback(self):
        msg = String()
        cpu_usage = psutil.cpu_percent(interval=None, percpu=False)
        load_ave = psutil.getloadavg()
        mem = psutil.virtual_memory()
        temps = psutil.sensors_temperatures()
        net = psutil.net_if_stats()

        msg.data = f"CPU:{cpu_usage}\nLoad Average:{load_ave}\nMemory:{mem}\nTemperatures:{temps}\nNet:{net}"
        self._publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    debug = DebugNode()
    rclpy.spin(debug)
    debug.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()