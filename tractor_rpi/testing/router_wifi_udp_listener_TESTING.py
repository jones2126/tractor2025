#!/usr/bin/env python3
"""
router_wifi_udp_listener_TESTING.py
====================================
Minimal test listener for wifi_publish_20260703.sh (runs on the GL router).
Just prints whatever arrives on UDP 6005, raw. No parsing, no CSV, nothing
fancy - purpose is only to confirm the router's script is actually sending
and the packets are arriving with the expected content.

Usage:
  python3 router_wifi_udp_listener_TESTING.py
"""

import socket

PORT = 6005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', PORT))
print(f"Listening for router WiFi status on UDP {PORT}... (Ctrl+C to stop)")

while True:
    data, addr = sock.recvfrom(1024)
    print(f"{addr[0]} -> {data.decode(errors='replace')}")
