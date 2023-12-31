#!/usr/bin/env python3
'''
input_keyboard.py

Take input from the keyboard and republish it to topic

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
import sys,tty,os,termios

 # Get current user terminal settings and save them for later
old_settings = termios.tcgetattr(sys.stdin)

# Enable cbreak mode. In cbreak mode characters typed by the user are immediately available
# to the program. This way we can extract keys without the user needing to press enter
tty.setcbreak(sys.stdin.fileno())

try:
    while True:
        # Read at most one byte (the length of one character) from stdin and decode it
        k = os.read(sys.stdin.fileno(), 1).decode()
        print(k)
except (KeyboardInterrupt, SystemExit):
    # Reset to original terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    # Make terminal sane again upon program exit
    os.system('stty sane')
