# thrust_box_bme280.py

# I think this belongs in init once its a ros node
import board
from adafruit_bme280 import basic as adafruit_bme280


i2c = board.I2C()  # uses board.SCL and board.SDA

# 0x76 is address for bme280 with bridged address pads

thrust_box_bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, 0x76)

print("thrust box")
print("\nTemperature: %0.1f C" % thrust_box_bme280.temperature)
print("Humidity: %0.1f %%" % thrust_box_bme280.humidity)
print("Pressure: %0.1f hPa" % thrust_box_bme280.pressure)
