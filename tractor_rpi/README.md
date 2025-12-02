# Tractor RPi 5 Main Code

This folder contains Python scripts used on the Raspberry Pi side of the
tractor.

## RTCM and Heading Server

`rtcm_server.py` forwards RTCM correction data to the base F9P receiver while
reading position and heading information from both receivers.  See the Miro Board for more details.

## teensy_serial_bridge.py

`teensy_serial_bridge_{date}.py` This bridge reads serial data from the Teensy microcontroller and broadcasts key system status via UDP at 5 Hz for monitoring and control.  See the Miro Board for more details.

## led_status_controller.py

`led_status_controller.py` This script uses the I2C port on the RPi to communicate with the 16-channel PWM controller.  That PWM controller controls 5 LED drivers that are used to control the Red, Green, Blue and Yellow LED light tower to provide various status signals. See the Miro Board for more details.


