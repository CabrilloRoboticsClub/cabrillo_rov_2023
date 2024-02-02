#!/usr/bin/env python3
import rclpy 
import psutil
import time
from psutil._common import bytes2human
from rclpy.node import Node 
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from seahawk_msgs.msg import Debug


class DebugNode(Node):
    def __init__(self):
        super().__init__("debug")
        self._publisher = self.create_publisher(Debug, "debug_info", 10)
        self.timer = self.create_timer(0.5, self.pub_callback) 
        net = psutil.net_io_counters()
        self.sent = net.bytes_sent
        self.recv = net.bytes_recv
    
    def pub_callback(self):
        msg = Debug()

        # Grab CPU usage, load average, and memory usage percent
        cpu_usage = psutil.cpu_percent(interval=None, percpu=False)
        load_ave = psutil.getloadavg()
        mem = psutil.virtual_memory().percent

        # Grabs CPU temps and averages temperature across cores
        temp_all = psutil.sensors_temperatures()["cpu_thermal"]
        temp_ave = sum([temp_all[i][1] for i in range(len(temp_all))])/len(temp_all)


        # Gets Net Stats
        net = psutil.net_io_counters()
        curr_sent = net.bytes_sent
        curr_recv = net.bytes_recv

        sent = bytes2human(curr_sent - self.sent)
        recv = bytes2human(curr_recv - self.recv)

        self.sent = curr_sent
        self.recv = curr_recv

        # msg.data = f"CPU: {cpu_usage}%\nLoad Average: {load_ave}\nMemory: {mem}%\nTemperatures: {temp_ave}°C\nSent: {sent}\nReceived: {recv}"
        msg.cpu_temperature = temp_ave
        msg.cpu_usage = cpu_usage
        msg.memory_usage = mem
        msg.net_sent = sent
        msg.net_recv = recv
        self._publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    debug = DebugNode()
    rclpy.spin(debug)
    debug.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()