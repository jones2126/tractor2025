#!/usr/bin/env python3
"""
gps_udp_listener.py – diagnostic tool for GPS/RTK state monitoring

UPDATED: Displays numSV, HDOP, diff_age fields from rtcm_server.
FIXED: Uses SO_REUSEPORT for reliable port sharing with other listeners.

Listens on UDP port 6002, prints compact status line per message,
and reports rate + quality summary every 5 seconds.
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
        # NEW: Track fix quality distribution
        self.fix_counts = {}
        self.sv_values = deque(maxlen=WINDOW_SIZE)
        self.hdop_values = deque(maxlen=WINDOW_SIZE)

    def add(self, ts: float, msg: dict):
        self.timestamps.append(ts)
        self.total_msgs += 1
        # NEW: Track fix distribution and quality metrics
        fix = msg.get('fix_quality', 'Unknown')
        self.fix_counts[fix] = self.fix_counts.get(fix, 0) + 1
        if msg.get('numSV') is not None:
            self.sv_values.append(msg['numSV'])
        if msg.get('hdop') is not None:
            self.hdop_values.append(msg['hdop'])

    def report(self, now: float) -> bool:
        """Print rate report if it's time and we have enough data."""
        if now - self.last_report < REPORT_INTERVAL:
            return False

        if len(self.timestamps) < 2:
            self.last_report = now
            return False

        ts_list = list(self.timestamps)
        window_secs = ts_list[-1] - ts_list[0]
        rate = len(ts_list) / window_secs if window_secs > 0 else 0.0

        intervals = [b - a for a, b in zip(ts_list, ts_list[1:])]
        min_int = min(intervals)
        max_int = max(intervals)

        print("\n=== GPS UDP RATE REPORT ===")
        print(f"Time               : {datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')}")
        print(f"Total messages     : {self.total_msgs}")
        print(f"Current rate       : {rate:.2f} Hz")
        print(f"Interval (min/max) : {min_int*1000:.1f} / {max_int*1000:.1f} ms")

        # NEW: Fix quality distribution
        if self.fix_counts:
            fix_str = ", ".join(f"{k}: {v}" for k, v in sorted(self.fix_counts.items()))
            print(f"Fix distribution   : {fix_str}")

        # NEW: Satellite and HDOP summary
        if self.sv_values:
            sv_list = list(self.sv_values)
            print(f"Satellites (SV)    : min={min(sv_list)}, max={max(sv_list)}, last={sv_list[-1]}")
        if self.hdop_values:
            hd_list = list(self.hdop_values)
            print(f"HDOP               : min={min(hd_list):.2f}, max={max(hd_list):.2f}, last={hd_list[-1]:.2f}")

        print("==========================\n")

        # Reset per-interval counters
        self.fix_counts.clear()
        self.last_report = now
        return True


# ----------------------------------------------------------------------
def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # CHANGED: Use SO_REUSEPORT (not just SO_REUSEADDR) for actual UDP port sharing on Linux
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)  # NEW ~line 92
    except AttributeError:
        pass  # SO_REUSEPORT not available on all platforms

    try:
        sock.bind(("127.0.0.1", UDP_PORT))
        print(f"Listening for GPS data on UDP port {UDP_PORT} (SO_REUSEPORT enabled)...")
    except OSError as e:
        if e.errno == 98:
            print(f"Port {UDP_PORT} in use. Try: sudo systemctl stop teensy-bridge led-status")
            return
        else:
            raise

    tracker = RateTracker()
    msg_count = 0
    # Print first 5 messages verbosely, then compact
    VERBOSE_COUNT = 5
    print("Ready. First 5 messages shown in full, then compact + rate every 5s.\n")

    while True:
        try:
            data, addr = sock.recvfrom(4096)
            now = time.time()

            try:
                msg = json.loads(data.decode())
            except json.JSONDecodeError:
                continue

            msg_count += 1

            # NEW: Compact one-line status showing diagnostic fields
            if msg_count <= VERBOSE_COUNT:
                # Full dump for first few messages
                print(json.dumps(msg, indent=2))
            elif msg_count % 20 == 0:
                # Compact line every 20th message (~1/sec at 20Hz)
                fix = msg.get('fix_quality', '?')
                sv = msg.get('numSV', '?')
                hdop = msg.get('hdop')
                diff = msg.get('diff_age')
                head = msg.get('headValid', '?')
                carrier = msg.get('carrier', '?')
                hdop_str = f"{hdop:.1f}" if hdop is not None else "?"
                diff_str = f"{diff:.1f}s" if diff is not None else "?"
                print(f"fix={fix:12s} SV={sv:>2} HDOP={hdop_str:>5} diff_age={diff_str:>6} head={head} carrier={carrier}")

            tracker.add(now, msg)
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
