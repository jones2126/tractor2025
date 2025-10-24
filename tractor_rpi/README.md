# Tractor Python Utilities

This folder contains Python scripts used on the Raspberry Pi side of the
tractor project.

## RTCM and Heading Server

`rtcm_server.py` forwards RTCM correction data to the base F9P receiver while
reading position and heading information from both receivers.  The latest
latitude, longitude, fix quality and heading are broadcast as JSON over UDP on
port `4242`.

Example usage:

```bash
python3 rtcm_server.py
```

Any program that needs navigation data can subscribe to the UDP stream.  The
script `testing/gps_udp_listener.py` demonstrates how to receive and display
these messages.

```bash
python3 testing/gps_udp_listener.py
```
