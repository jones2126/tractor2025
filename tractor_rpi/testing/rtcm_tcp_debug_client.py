"""
rtcm_tcp_debug_client.py
========================
Minimalist script to connect to a TCP server publishing RTCM3 messages
(IP: 192.168.1.95, port: 6001), parse the binary stream, identify message types,
and report publishing status and rates (total + per-type).

Features:
- Auto-reconnect on disconnect/error
- Parses RTCM3 frames (searches for 0xD3 preamble, extracts message type)
- Prints summary every 5 seconds: messages/sec, per-type breakdown
- No file logging, no serial forwarding — pure debugging focus

Usage:
    python rtcm_tcp_debug_client.py
"""

import socket
import time
from collections import Counter

HOST = "192.168.1.95"
PORT = 6001
RECV_SIZE = 4096
PRINT_INTERVAL = 5.0  # seconds between status reports

def connect():
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((HOST, PORT))
            print(f"Connected to {HOST}:{PORT}")
            return sock
        except Exception as e:
            print(f"Connection failed ({e}). Retrying in 5 seconds...")
            time.sleep(5)

def main():
    buffer = b""
    total_messages = 0
    total_bytes = 0
    start_time = time.time()
    interval_counter = Counter()
    last_print = start_time

    sock = connect()

    while True:
        try:
            data = sock.recv(RECV_SIZE)
            if not data:
                print("Server closed connection. Reconnecting...")
                sock.close()
                sock = connect()
                buffer = b""  # clear buffer on reconnect
                continue

            buffer += data
            total_bytes += len(data)

            pos = 0
            while pos + 6 <= len(buffer):
                if buffer[pos] != 0xD3:
                    pos += 1
                    continue

                if pos + 2 >= len(buffer):
                    break

                length = ((buffer[pos + 1] & 0x3F) << 8) | buffer[pos + 2]
                frame_length = length + 6

                if pos + frame_length > len(buffer):
                    break

                payload_start = pos + 3
                payload = buffer[payload_start:payload_start + length]

                msg_type = 0
                if length >= 2:
                    msg_type = ((payload[0] << 4) | (payload[1] >> 4)) & 0xFFF
                    interval_counter[msg_type] += 1
                    total_messages += 1

                pos += frame_length

            buffer = buffer[pos:]

            now = time.time()
            if now - last_print >= PRINT_INTERVAL:
                interval_elapsed = now - last_print
                interval_msgs = sum(interval_counter.values())
                interval_rate = interval_msgs / interval_elapsed if interval_elapsed > 0 else 0

                total_elapsed = now - start_time
                overall_rate = total_messages / total_elapsed if total_elapsed > 0 else 0
                bytes_rate = total_bytes / total_elapsed if total_elapsed > 0 else 0

                print(f"\n=== [{time.strftime('%H:%M:%S')}] ===")
                print(f"Interval ({interval_elapsed:.1f}s): {interval_msgs} msgs, {interval_rate:.2f} msg/s")
                if interval_msgs > 0:
                    print("Types this interval:")
                    for mt, cnt in sorted(interval_counter.items()):
                        print(f"  {mt}: {cnt} ({cnt / interval_elapsed:.2f}/s)")
                else:
                    print("No valid RTCM3 messages in this interval (check stream/format)")
                print(f"Overall: {total_messages} msgs, {overall_rate:.2f} msg/s, {bytes_rate:.0f} B/s")

                interval_counter.clear()
                last_print = now

        except Exception as e:
            print(f"Error ({e}). Reconnecting...")
            try:
                sock.close()
            except:
                pass
            sock = connect()
            buffer = b""

if __name__ == "__main__":
    print("RTCM TCP debug client started (Ctrl+C to exit)")
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting.")