#!/usr/bin/env python3
"""
simple_cmd_vel_tester.py
========================
Send cmd_vel over UDP → bridge → Teensy → echo back.
Shows live echo count and pending commands.
"""

import socket
import json
import time
import threading
import sys

# ------------------------------------------------------------
# CONFIG – same as bridge
UDP_CMD_PORT   = 6004          # bridge listens here
UDP_STATUS_PORT = 6003         # bridge broadcasts here
BROADCAST_IP   = "255.255.255.255"

# ------------------------------------------------------------
class SimpleTester:
    def __init__(self):
        # ---- send socket (broadcast) ----
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # ---- receive socket (status) ----
        self.status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.status_sock.bind(('', UDP_STATUS_PORT))
        self.status_sock.settimeout(0.5)

        self.sent = 0
        self.echoed = 0
        self.last_echo = 0

        print("Simple CMD_VEL tester ready")
        print("  → Send to 255.255.255.255:6004")
        print("  ← Listen on :6003")
        print()

    # --------------------------------------------------------
    def send(self, linear_x=0.0, angular_z=0.0):
        payload = {
            "linear_x": round(linear_x, 4),
            "angular_z": round(angular_z, 4),
            "timestamp": time.time()
        }
        json_data = json.dumps(payload)
        data = (json_data + "\n").encode()  # ← CRITICAL: ADD \n
        self.cmd_sock.sendto(data, (BROADCAST_IP, UDP_CMD_PORT))  # ← FIXED VARIABLE
        self.sent += 1
        print(f"[{self.sent:4d}] → linear_x={linear_x:+.3f}  angular_z={angular_z:+.3f}")

    # --------------------------------------------------------
    def _status_loop(self):
        while True:
            try:
                raw, _ = self.status_sock.recvfrom(4096)
                status = json.loads(raw.decode())
                if 'cmd_vel' in status:
                    echoed = status['cmd_vel'].get('commands_echoed', 0)
                    if echoed != self.last_echo:
                        self.echoed = echoed
                        self.last_echo = echoed
                        pending = self.sent - self.echoed
                        print(f"  ← ECHO #{echoed}  (pending={pending})")
            except socket.timeout:
                continue
            except Exception as e:
                print(f"status recv error: {e}", file=sys.stderr)

    # --------------------------------------------------------
    def run(self):
        # start background status listener
        threading.Thread(target=self._status_loop, daemon=True).start()

        print("Enter commands (or blank line to quit):")
        print("  <linear_x> <angular_z>   e.g. 0.5 0.2")
        print()

        try:
            while True:
                line = input("> ").strip()
                if not line:
                    break
                parts = line.split()
                if len(parts) != 2:
                    print("need two numbers")
                    continue
                try:
                    lx = float(parts[0])
                    az = float(parts[1])
                except ValueError:
                    print("not numbers")
                    continue
                self.send(lx, az)
                time.sleep(0.02)          # ~50 Hz max, safe for bridge
        except (KeyboardInterrupt, EOFError):
            pass

        print("\nSummary:")
        print(f"  Sent    : {self.sent}")
        print(f"  Echoed  : {self.echoed}")
        print(f"  Lost    : {self.sent - self.echoed}")

# ------------------------------------------------------------
if __name__ == "__main__":
    SimpleTester().run()