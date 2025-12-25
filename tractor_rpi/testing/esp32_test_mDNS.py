"""
Minimal Python TCP client to test mDNS resolution and connection
Connects to rtcm-esp32.local:6009 and prints received data
"""

import socket
import time

HOST = "rtcm-esp32.local"  # mDNS hostname
PORT = 6009

def main():
    print(f"Attempting to connect to {HOST}:{PORT} ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((HOST, PORT))
        print(f"Connected successfully using mDNS resolution!")
        sock.settimeout(5)  # For non-blocking read with timeout

        while True:
            try:
                data = sock.recv(1024)
                if not data:
                    print("Connection closed by server.")
                    break
                print("Received:", data.decode().strip())
            except socket.timeout:
                continue  # No data yet, keep looping
            except Exception as e:
                print("Error receiving data:", e)
                break
    except Exception as e:
        print(f"Connection failed: {e}")
        print("   - Check that the ESP32 is running the sketch and on the same network")
        print("   - On Raspberry Pi, ensure avahi-daemon is installed/running: sudo apt install avahi-daemon")
    finally:
        sock.close()

if __name__ == "__main__":
    while True:  # Auto-reconnect loop
        main()
        print("Reconnecting in 5 seconds...")
        time.sleep(5)