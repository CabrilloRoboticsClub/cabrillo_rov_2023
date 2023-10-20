#!/usr/bin/env python3
import rclpy 
import psutil
import time
from psutil._common import bytes2human
from rclpy.node import Node 
from std_msgs.msg import String
from geometry_msgs import Vector3


class DebugNode(Node):
    def __init__(self):
        super().__init__("debug")
        self._publisher = self.create_publisher(Vector3, "debug_info", 10)
        self.timer = self.create_timer(0.5, self.pub_callback) 
    
    def pub_callback(self):
        msg = Vector3()

        # Grab CPU usage, load average, and memory usage percent
        cpu_usage = psutil.cpu_percent(interval=None, percpu=False)
        load_ave = psutil.getloadavg()
        mem = psutil.virtual_memory().percent

        # Grabs CPU temps and averages temperature across cores
        temp_all = psutil.sensors_temperatures()["cpu_thermal"]
        temp_ave = sum([temp_all[i][1] for i in range(len(temp_all))])/len(temp_all)


        # Gets Net Stats, wait for 0.25s to catch traffic
        net = psutil.net_io_counters()
        sent_before = net.bytes_sent
        recv_before = net.bytes_recv
        time.sleep(0.25)
        net = psutil.net_io_counters()
        sent_after = net.bytes_sent
        recv_after = net.bytes_recv

        # Converts to b/kb/mb based on size
        sent = bytes2human(sent_after - sent_before)
        recv = bytes2human(recv_after - recv_before)

        # msg.data = f"CPU: {cpu_usage}%\nLoad Average: {load_ave}\nMemory: {mem}%\nTemperatures: {temp_ave}°C\nSent: {sent}\nReceived: {recv}"
        msg[0] = temp_ave
        msg[1] = cpu_usage
        msg[2] = mem
        self._publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    debug = DebugNode()
    rclpy.spin(debug)
    debug.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()