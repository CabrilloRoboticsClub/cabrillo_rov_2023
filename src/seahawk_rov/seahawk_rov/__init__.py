'''
seahawk_rov/__init__.py

this is the init for the main node that runs on the ROV
it imports all the classes to make them available to main

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

# sensor classes
from .bme280 import BME280
from .logic_tube_bme280 import LogicTubeBME280
from .logic_tube_bno085 import LogicTubeBNO085
from .thrust_box_bme280 import ThrustBoxBME280

# output classes
from .logic_tube_motor import LogicTubeMotor
from .logic_tube_servo import LogicTubeServo
from .thrust_box_servo import ThrustBoxServo

# helper functions
from .helper_functions import float_to_pwm
from .helper_functions import clamp

from .__main__ import main
