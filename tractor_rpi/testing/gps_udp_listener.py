#!/usr/bin/env python3
"""
gps_udp_listener.py – multi-subscriber safe + robust live rate reporting

Listens on UDP port 6002 (shared with led_status_controller.py, etc.)
Prints every JSON message and reports actual receive rate every 5 seconds.
"""

import json
import socket
import time
from collections import deque
from datetime import datetime, timezone

UDP_PORT = 6002

# ----------------------------------------------------------------------
# Rate-tracking helpers
# ----------------------------------------------------------------------
REPORT_INTERVAL = 5.0          # seconds between rate reports
WINDOW_SIZE = int(REPORT_INTERVAL * 50)   # enough for >40 Hz

class RateTracker:
    def __init__(self):
        self.timestamps = deque(maxlen=WINDOW_SIZE)
        self.total_msgs = 0
        self.last_report = 0.0

    def add(self, ts: float):
        self.timestamps.append(ts)
        self.total_msgs += 1

    def report(self, now: float) -> bool:
        """Print rate report if it's time and we have enough data."""
        if now - self.last_report < REPORT_INTERVAL:
            return False

        if len(self.timestamps) < 2:
            # Not enough data for a meaningful rate
            self.last_report = now
            return False

        # Convert deque to list only when needed (avoids slice error)
        ts_list = list(self.timestamps)
        window_start = ts_list[0]
        window_end   = ts_list[-1]
        window_secs  = window_end - window_start
        rate = len(ts_list) / window_secs if window_secs > 0 else 0.0

        # Compute intervals
        intervals = [b - a for a, b in zip(ts_list, ts_list[1:])]
        min_int = min(intervals)
        max_int = max(intervals)

        print("\n=== GPS UDP RATE REPORT ===")
        print(f"Time               : {datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')}")
        print(f"Total messages     : {self.total_msgs}")
        print(f"Messages in window : {len(ts_list)}")
        print(f"Window duration    : {window_secs:.3f} s")
        print(f"Current rate       : {rate:.2f} Hz")
        print(f"Min interval       : {min_int*1000:6.2f} ms")
        print(f"Max interval       : {max_int*1000:6.2f} ms")
        print("==========================\n")

        self.last_report = now
        return True


# ----------------------------------------------------------------------
def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        sock.bind(("", UDP_PORT))
        print(f"Listening for GPS data on UDP port {UDP_PORT} (multi-subscriber mode)...")
    except OSError as e:
        if e.errno == 98:
            print(f"Port {UDP_PORT} already in use – this is OK (shared with other listeners).")
        else:
            raise

    tracker = RateTracker()
    print("Ready. Printing messages and rate every 5 seconds...\n")

    while True:
        try:
            data, addr = sock.recvfrom(4096)
            now = time.time()

            # --- Print the raw JSON message ---
            try:
                msg = json.loads(data.decode())
                #print(msg)
            except json.JSONDecodeError:
                continue  # skip malformed packets

            # --- Track rate ---
            tracker.add(now)
            tracker.report(now)

        except KeyboardInterrupt:
            print("\nShutting down...")
            break
        except Exception as e:
            print(f"Unexpected error: {e}")
            time.sleep(0.1)

    sock.close()


if __name__ == "__main__":
    main()