# Setup Procedure

Person 1 and person 2 work in parallel

## Person 1 (Ciaran or Orion)

1. Open the Deck box on table in front of the pilot.

1. Look through the Lexan sheet on the deck box and check for any connectors that have fallen out.

    * [wiring diagram](example.com)

1. Screw the wifi antennas into their RP-SMA jacks on the deck box.

1. Turn on the keyboard with the switch on the top edge.

1. Plug the monitor into its power supply (USB-C cable) on the deck box.

1. If WAN is available, connect it to the WAN port on the deck box.

1. Plug the IEC-C13 Cable into the 120V power supplied by MATE.

1. Plug the IEC-C13 cable into the IEC-C14 "POWER IN" port on the deck box.

1. Flip the "POWER IN" switch to the ON position on the deck box.

## Person 2 (Tether Manager)

1. Connect the tether to the ROV.
    * Ethernet
    * Pneumatic Line
    * Power Connector

1. Connect the tether SBS50 to the 48v PSU

1. Connect the tether Ethernet to the "ROV LAN" port on the deck box

1. Connect the tether pneumatic line to the compressor

1. Plug the IEC-C19 cable for the 48V PSU into the 120V power supplied by mate

1. Plug the IEC-C19 cable into the IEC-C20 socket on the 48V PSU

1. Flip the "INPUT SW" to "I" on the 48V PSU

1. Flip the "OUTPUT SW" to "ON" on the 48V PSU

1. Uncoil the XBOX Controller cable.

1. Plug the XBOX Controller into "REAR USB 2"

## Person 1 (Ciaran or Orion) (part 2)

1. Wait for the monitor to show the Ubuntu Desktop.

1. On the Keyboard use the `ctrl+alt+t` keyboard shortcut to open a Terminal window

1. Open 2 more tabs with the new tab button in the top left so there are 3 tabs are open

1. in the first tab launch the deck

```console
source ~/cabrillo_rov_2023/install/setup.bash && ros2 launch seahawk_deck deck.launch.py
```

1. In the second tab ssh into the rov

```console
ssh ubuntu@SeaHawk-ROV.lan
```

1. In the second tab launch the ROV

```console
source ~/cabrillo_rov_2023/install/setup.bash && ros2 launch seahawk_rov rov.launch.py
```

1. in the third tab open rqt

```console
source ~/cabrillo_rov_2023/install/setup.bash && rqt
```

## YOU ARE READY TO DRIVE THE ROV
