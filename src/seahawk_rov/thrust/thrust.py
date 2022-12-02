'''
thrust.py

this module takes the twist messages and translates them to servo angles for thrust box pwm.

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

# the ros client library for python
import rclpy

# importing the idea of node so I can define a subscriber
from rclpy.node import Node

# twish messages are what we are taking as input
from geometry_msgs.msg import Twist


### thruster naming
#
# FRU = Front Right Up
# FLU = Front Left Up
# FRD = Front Right Down
# FLD = Front Left Down
# RRU = Rear Right Up
# RLU = Rear Left Up
# RRD = Rear Rght Down
# RLD = Rear Left Down