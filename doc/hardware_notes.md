# Hardware Notes

## i2c Addresses

### Sensors

#### BME280 (unbridged)
0x77

#### BME280 (bridged)
0x76

### PCA9685 PWM Drivers

Note that address 0x70 is the "all call" address for the controller chip on the HAT. All boards will respond to address 0x70 - regardless of the address jumper settings.

#### Servo Driver Hat
0x40

#### Motor Driver Hat
0x60

#### Servo Driver (A0 Bridged)
0x41