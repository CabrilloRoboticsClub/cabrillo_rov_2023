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

*  Sensors

* * BME280 (unbridged)
* * * 0x77

* * BME280 (bridged)
* * * 0x76

* PCA9685 PWM Drivers

Note that address 0x70 is the "all call" address for the controller chip on the HAT. All boards will respond to address 0x70 - regardless of the address jumper settings.

* *  Servo Driver Hat
* * * 0x40

* *  Motor Driver Hat
* * * 0x60

* *  Servo Driver (A0 Bridged)
* * * 0x41