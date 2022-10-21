#thrust.py
'''
cabrillo_rov_2023
thrust module

this module takes the twist messages and translates them to servo angles for thrust box pwm.
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