#!/usr/bin/env python3
"""
debug.py

Publish RPi4 Debug info to /debug_info

Copyright (C) 2023-2024 Cabrillo Robotics Club

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
"""
import rclpy 
import psutil
from psutil._common import bytes2human
from rclpy.node import Node 
from seahawk_msgs.msg import DebugInfo

class Debug(Node):
    """
    Reads system information from RPi4 and publishes it.
    """
    def __init__(self):
        super().__init__("debug")
        self._publisher = self.create_publisher(DebugInfo, "debug_info", 10)
        self.timer = self.create_timer(0.5, self.pub_callback) 
        net = psutil.net_io_counters()
        self.sent = net.bytes_sent
        self.recv = net.bytes_recv
    
    def pub_callback(self):
        msg = DebugInfo()

        # Grabs CPU temps and averages temperature across cores
        temp_all = psutil.sensors_temperatures()["cpu_thermal"]
        msg.cpu_temperature = sum(temp[1] for temp in temp_all) / len(temp_all)

        # Grab CPU usage, load average, and memory usage percent
        msg.cpu_usage = psutil.cpu_percent(interval=None, percpu=False)
        msg.memory_usage = psutil.virtual_memory().percent

        # Gets Net Stats
        net = psutil.net_io_counters()
        curr_sent = net.bytes_sent
        curr_recv = net.bytes_recv

        sent = bytes2human(curr_sent - self.sent)
        recv = bytes2human(curr_recv - self.recv)

        self.sent = curr_sent
        self.recv = curr_recv

        msg.net_sent = sent
        msg.net_recv = recv

        self._publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    debug_node = Debug()
    rclpy.spin(debug_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()