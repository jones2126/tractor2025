#!/usr/bin/env python3
"""
router_wifi_tcp_listener_TESTING.py
=====================================
Minimal test listener for wifi_publish.sh (runs on the GL router).

CHANGED (7/3/26): switched from UDP to TCP. The GL router's BusyBox nc
build only supports a bare TCP pipe (`nc IPADDR PORT`) - confirmed via
testing that it has no -u (UDP) or -w (timeout) flags at all.

Accepts one connection at a time, reads whatever the router sends, prints
it, then closes and waits for the next connection. No parsing, no CSV -
purpose is only to confirm the router's script is sending and packets are
arriving with the expected content.

Usage:
  python3 router_wifi_tcp_listener_TESTING.py
"""

import socket

PORT = 6005

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('', PORT))
server.listen(1)
print(f"Listening for router WiFi status on TCP {PORT}... (Ctrl+C to stop)")

while True:
    conn, addr = server.accept()
    try:
        data = conn.recv(1024)
        print(f"{addr[0]} -> {data.decode(errors='replace')}")
    finally:
        conn.close()
