# logic_tube_bme280.py

# I think this belongs in init once its a ros node
import board
from adafruit_bme280 import basic as adafruit_bme280


i2c = board.I2C()  # uses board.SCL and board.SDA

# 0x77 is address for bme280 with un-bridged address pads

logic_tube_bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, 0x77)

print("logic tube")
print("\nTemperature: %0.1f C" % logic_tube_bme280.temperature)
print("Humidity: %0.1f %%" % logic_tube_bme280.humidity)
print("Pressure: %0.1f hPa" % logic_tube_bme280.pressure)
