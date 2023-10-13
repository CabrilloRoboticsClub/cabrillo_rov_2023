import rclpy 
from rclpy import Node

# ros messages
from sensor_msgs.msg import FluidPressure

# import the bme280 circuit python sensor library
from adafruit_bme280 import basic as adafruit_bme280

# BME280
class PressureSensor(Node):
    def __init__(self, i2c):
        # constants
        self._i2c_bus = i2c
        self._i2c_addr = 0x77,
        self._frame_id = "base_link"

        # instantiate the sensor
        self.bme = adafruit_bme280.Adafruit_BME280_I2C(i2c=self.i2c_bus, address=self.i2c_addr)
        
        # create the publisher to the topic logic_tube/pressure with message type FluidPressure
        self.pressure_sensor_pub = self.create_publisher(msg_type=FluidPressure, topic="logic_tube/pressure", qos_profile=10)

        # Timer that calls the pub_callback function every 5 seconds
        # TODO: Add callback group
        self.create_timer(timer_period_sec=5, callback=self.pub_callback)

    def pub_callback():
        pass

def main(args=None):
    rclpy.init(args=args)
    # Code here
    rclpy.shutdown()

if __name__ == "__main__":
    main()