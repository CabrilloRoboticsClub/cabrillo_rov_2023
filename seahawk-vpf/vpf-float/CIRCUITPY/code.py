# Cabrillo Robotics Club
# Vertical Profiling Float
# Float

# python hardware interfaces
import board
import busio
import digitalio
import os

# lora radio library
import adafruit_rfm9x

# get board details
board_type = os.uname().machine

# instanciate the spi interface
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)


#
# LoRa Radio Wing SETUP
#

# set the Chip Select and Reset pins based on board type
if "Particle Xenon" in board_type:
    CS = digitalio.DigitalInOut(board.D2)
    RESET = digitalio.DigitalInOut(board.D3)
elif "RFM9x" in board_type:
    CS = digitalio.DigitalInOut(board.RFM9X_CS)
    RESET = digitalio.DigitalInOut(board.RFM9X_RST)
else:
    CS = digitalio.DigitalInOut(board.D5)
    RESET = digitalio.DigitalInOut(board.D6)

# set the radio frequency to 915mhz
RADIO_FREQ_MHZ = 915.0 

# instanciate the lora radio in 915mhz mode
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

# set my lora node ID
rfm9x.node = 100

# set the destination lora node ID
# destination is deck
rfm9x.destination = 50


rfm9x.send("Hello World")