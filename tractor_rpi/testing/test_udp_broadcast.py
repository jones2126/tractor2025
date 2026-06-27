#!/usr/bin/env python3
"""
Minimal UDP broadcast test.
Sends incrementing counter packets every 0.2s.
"""

import socket
import time
import json

# === CONFIGURE THESE FOR YOUR TEST ===
INTERFACE_IP = '192.168.1.213'          # Your wlan0 IP
BROADCAST_IP = '192.168.1.255'          # wlan0 broadcast address
PORT = 6003
# =====================================

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)

# Bind to specific interface IP (often helps)
sock.bind((INTERFACE_IP, 0))

print(f"Starting UDP broadcast test on {BROADCAST_IP}:{PORT}")
print("Packets should appear in tcpdump immediately.")
print("Press Ctrl+C to stop.\n")

counter = 0
while True:
    try:
        payload = {
            "counter": counter,
            "timestamp": time.time(),
            "test": "udp_broadcast"
        }
        data = json.dumps(payload).encode('utf-8')

        # Send twice + small yield (forces kernel to flush)
        sock.sendto(data, (BROADCAST_IP, PORT))
        time.sleep(0.0005)
        sock.sendto(data, (BROADCAST_IP, PORT))

        print(f"Sent packet #{counter} ({len(data)} bytes)")
        counter += 1
        time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nStopping...")
        break
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(1)

sock.close()
print("Done")