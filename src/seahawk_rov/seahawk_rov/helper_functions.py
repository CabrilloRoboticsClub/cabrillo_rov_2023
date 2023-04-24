'''
seahawk_rov/helper_functions.py

helper functions to speed up code in other classes

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

# linear interpolation helper function
def float_to_pwm(old_value:float, new_min:int=0, new_max:int=3000)->int:
    '''linear interpolate helper function'''

    old_min:float = -1.0
    old_max:float = 1.0

    old_range = old_max - old_min
    new_range = new_max - new_min

    new_value = (((old_value - old_min) * new_range) / old_range) + new_min
    return int(new_value)


# clamp helper function
# helps keep us from starting fires
def clamp(num, minimum, maximum):
  '''clamp helper function'''
  
  # gives the highest value of the two.
  # aka if the minimum value is higher than the input num than the minimum value is returned
  # removes values lower than the minimum
  maximised = max(minimum, num)

  # gives the minimum value of the two
  # aka if the maximised value is lower than the input num than the maximum value is returned
  # removes values higher than the maximum
  minimised = min(maximised, maximum)

  return minimised

# deadzone helper function
# if within range set value to something; else do nothing
def deadzone(num, min, max, value=0):
   if min <= num <= max:
      return value
   return num