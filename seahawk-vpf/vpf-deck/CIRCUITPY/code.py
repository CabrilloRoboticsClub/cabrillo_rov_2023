# Cabrillo Robotics Club
# Vertical Profiling Float
# Deck

# python hardware interfaces
import board
import busio
import digitalio
import os

# lora radio library
import adafruit_rfm9x

# oled display library
import displayio
import terminalio
import adafruit_displayio_sh1107
from adafruit_display_text import label
from adafruit_display_shapes.rect import Rect

# get board details
board_type = os.uname().machine

# instantiate the spi interface
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# instantiate the i2c interface
i2c = board.I2C()

#
# LoRa Radio Feather SETUP
#

# set the Chip Select and Reset pins based on board type
if "Particle Xenon" in board_type:
    CS = digitalio.DigitalInOut(board.D2)
    RESET = digitalio.DigitalInOut(board.D3)
elif "RFM9x" in board_type:
    cs = digitalio.DigitalInOut(board.RFM9X_CS)
    reset = digitalio.DigitalInOut(board.RFM9X_RST)
else:
    CS = digitalio.DigitalInOut(board.D5)
    RESET = digitalio.DigitalInOut(board.D6)

# set the radio frequency to 915mhz
RADIO_FREQ_MHZ = 915.0 

# instantiate the lora radio in 915mhz mode
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

# set my node ID
rfm9x.node = 50

#
# OLED Wing SETUP
#

# reset display to cleanly handle soft reset
displayio.release_displays()

# instantiate a displayio display bus for the oled screen
display_bus = displayio.I2CDisplay(i2c, device_address=0x3c)

# instantiate the display
display = adafruit_displayio_sh1107.SH1107(
    display_bus, width=128, height=64, rotation=0
)

# create a layer for switching to the display
display_group = displayio.Group()
# show the layer on the display
display.show(display_group)


#
# Display Header
#

# draw outline for club name
display_group.append(
    Rect(
        0, 
        0, 
        128, 
        16, 
        fill=0x000000,
        outline=0xFFFFFF
    )   
)

# Draw the club name at the top of the display
display_group.append(
    label.Label(
        terminalio.FONT, 
        text="Cabrillo Robotics", 
        color=0xFFFFFF, 
        x=14, 
        y=8
    )
)


#
# Receive data from radio and display it
#

while True:

    # listen for a packet
    packet = rfm9x.receive()

    if packet is None:
        pass
    else:
        # blank screen to display new packet
        display_group.append(
            Rect(
                0,
                20,
                128,
                64,
                fill=0x000000
            )
        )
        # write packet data to screen
        display_group.append(
            label.Label(
                terminalio.FONT, 
                text=str(packet, "ascii"),
                color=0xFFFFFF,
                x=8,
                y=24
            )
        )