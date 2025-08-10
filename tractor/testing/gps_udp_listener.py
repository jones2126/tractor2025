#!/usr/bin/env python3
"""Example client for rtcm_server UDP output.

Listens on UDP port 4242 and prints received JSON messages.  This is useful for
verifying that ``rtcm_server.py`` is publishing navigation data.
"""

import json
import socket

UDP_PORT = 4242

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", UDP_PORT))
    print(f"Listening for GPS data on UDP port {UDP_PORT}...")
    while True:
        data, _ = sock.recvfrom(4096)
        try:
            msg = json.loads(data.decode())
        except json.JSONDecodeError:
            continue
        print(msg)

if __name__ == "__main__":
    main()
