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
        BYTE_TO_MEGABYTE = 1024 ^ 2

        msg = String()
        cpu_usage = psutil.cpu_percent(interval=None, percpu=False)
        load_ave = psutil.getloadavg()
        mem = psutil.virtual_memory().percent

        temp_all = psutil.sensors_temperatures()["coretemp"]
        temp_ave = sum([temp_all[i][1] for i in range(len(temp_all))])/len(temp_all)

        net = psutil.net_io_counters()

        sent = net[0] / BYTE_TO_MEGABYTE
        received = net[1] / BYTE_TO_MEGABYTE
        
        # self.get_logger().info(f"{sent}")
        # self.get_logger().info(f"{received}")
        self.get_logger().info(f"{net}")
        


        msg.data = f"CPU: {cpu_usage}%\nLoad Average: {load_ave}\nMemory: {mem}%\nTemperatures: {temp_ave}°C\nNet: {net}"
        self._publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    debug = DebugNode()
    rclpy.spin(debug)
    debug.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()