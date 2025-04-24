import socket
import base64
import serial
import time
import threading

# NTRIP server details (from your successful GNSS Viewer setup)
NTRIP_SERVER = "216.218.192.170"
NTRIP_PORT = 2101
MOUNTPOINT = "cmuairlab01"
USERNAME = "aej2126l@protonmail.com"
PASSWORD = ""  # No password, as confirmed by GNSS Viewer

# Serial port details for SkyTraq GPS (PX1172RDP)
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

# Global variables for RTK status
latest_gps_data = {
    "fix_type": None,
    "satellites": None,
    "hdop": None,
    "altitude": None,
    "last_report_time": 0
}

def parse_gpgga(sentence):
    """Parse a $GPGGA NMEA sentence and extract RTK status."""
    try:
        parts = sentence.strip().split(',')
        if parts[0] != "$GPGGA" or len(parts) < 10:
            return

        # Extract relevant fields
        fix_type = int(parts[6]) if parts[6] else 0  # Fix type (0: No Fix, 1: GPS Fix, 4: RTK Fixed, 5: RTK Float)
        satellites = int(parts[7]) if parts[7] else 0  # Number of satellites
        hdop = float(parts[8]) if parts[8] else None  # Horizontal Dilution of Precision
        altitude = float(parts[9]) if parts[9] else None  # Altitude in meters

        # Update global status
        latest_gps_data["fix_type"] = fix_type
        latest_gps_data["satellites"] = satellites
        latest_gps_data["hdop"] = hdop
        latest_gps_data["altitude"] = altitude

    except (ValueError, IndexError) as e:
        print(f"Error parsing GPGGA sentence: {e}")

def report_rtk_status():
    """Report RTK status every 5 seconds."""
    while True:
        time.sleep(5)  # Report every 5 seconds
        fix_type = latest_gps_data["fix_type"]
        satellites = latest_gps_data["satellites"]
        hdop = latest_gps_data["hdop"]
        altitude = latest_gps_data["altitude"]

        if fix_type is None:
            print("No GPGGA data received yet")
            continue

        fix_status = {
            0: "No Fix",
            1: "GPS Fix",
            4: "RTK Fixed",
            5: "RTK Float"
        }.get(fix_type, f"Unknown Fix Type ({fix_type})")

        print(f"RTK Status: {fix_status}, Satellites: {satellites}, HDOP: {hdop}, Altitude: {altitude}m")

def read_nmea(gps_serial):
    """Read NMEA sentences from the GPS and parse them."""
    while True:
        try:
            # Read a line from the serial port (NMEA sentences are newline-terminated)
            line = gps_serial.readline().decode('ascii', errors='ignore').strip()
            if line.startswith("$GPGGA"):
                parse_gpgga(line)
        except Exception as e:
            print(f"Error reading NMEA: {e}")
            time.sleep(1)  # Prevent rapid error looping

def fetch_rtcm():
    """Fetch RTCM data from NTRIP server and send to GPS, while reading NMEA."""
    # Create a TCP socket for the NTRIP connection
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.settimeout(10)  # Timeout for connection

    try:
        # Connect to the NTRIP server
        client.connect((NTRIP_SERVER, NTRIP_PORT))
        print(f"Connected to {NTRIP_SERVER}:{NTRIP_PORT}")

        # Encode username:password for Basic Authentication
        auth = base64.b64encode(f"{USERNAME}:{PASSWORD}".encode()).decode()

        # Send NTRIP request headers
        headers = (
            f"GET /{MOUNTPOINT} HTTP/1.0\r\n"
            f"User-Agent: NTRIP/RTKLIB/2.4.3\r\n"  # Mimic RTKLIB user-agent
            f"Authorization: Basic {auth}\r\n"
            f"Accept: */*\r\n"
            f"Connection: close\r\n"
            f"\r\n"
        )
        client.send(headers.encode())
        print("Sent NTRIP request headers")

        # Receive the server response (expecting HTTP/1.0 200 OK)
        response = client.recv(1024).decode()
        if "ICY 200 OK" not in response:
            print("Failed to connect to NTRIP server:", response)
            raise Exception("NTRIP connection failed")

        print("NTRIP server connected successfully")

        # Open serial port to SkyTraq GPS
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as gps_serial:
            print(f"Opened serial port {SERIAL_PORT} at {BAUDRATE} baud")

            # Start a thread to read NMEA sentences
            nmea_thread = threading.Thread(target=read_nmea, args=(gps_serial,), daemon=True)
            nmea_thread.start()

            # Start a thread to report RTK status every 5 seconds
            report_thread = threading.Thread(target=report_rtk_status, daemon=True)
            report_thread.start()

            # Stream RTCM data to the GPS
            while True:
                # Receive RTCM data from the NTRIP server
                data = client.recv(1024)
                if not data:
                    print("No more data from NTRIP server")
                    break

                # Write RTCM data to the GPS
                gps_serial.write(data)
                print(f"Sent {len(data)} bytes to GPS")

                # Small delay to prevent overwhelming the GPS
                time.sleep(0.01)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        client.close()
        print("NTRIP connection closed")

if __name__ == "__main__":
    fetch_rtcm()