<!--
doc/hardware_notes.md

notes about hardware for software developement

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
-->

# Hardware Notes

## i2c Addresses

```console
ubuntu@SeaHawk:~$ i2cdetect -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: 40 41 -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: 60 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: 70 -- -- -- -- -- 76 77                         
ubuntu@SeaHawk:~$ 
```

* Sensors

* BNO085 (default)
      * 0x4a

* BNO085 (Pin DI High)
      * 0x4b

* BME280 (unbridged)
      * 0x77

  * BME280 (bridged)
    * 0x76

* PCA9685 PWM Drivers

    Note that address 0x70 is the "all call" address for the controller chip on the HAT. All boards will respond to address 0x70 - regardless of the address jumper settings.

  * Servo Driver Hat
    * 0x40

  * Motor Driver Hat
    * 0x60

  * Servo Driver (A0 Bridged)
    * 0x41
