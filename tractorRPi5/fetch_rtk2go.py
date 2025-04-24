import socket
import base64
import serial
import time
import threading
import traceback

# NTRIP server details
NTRIP_SERVER = "216.218.192.170"
NTRIP_PORT = 2101
MOUNTPOINT = "cmuairlab01"
USERNAME = "aej2126l@protonmail.com"
PASSWORD = ""

# Serial port details for SkyTraq GPS (PX1172RDP)
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

# Approximate location near Pittsburgh, PA (from your GNSS Viewer data: 40.2043°N, 80.7437°W)
LATITUDE = "4020.729945,N"  # 40°20.729945' N
LONGITUDE = "08007.729845,W"  # 80°07.729845' W
ALTITUDE = 291.72  # Altitude in meters from GNSS Viewer

# Global variables for RTK status
latest_gps_data = {
    "fix_type": None,
    "satellites": None,
    "hdop": None,
    "altitude": None,
    "last_report_time": 0
}

def calculate_checksum(sentence):
    """Calculate NMEA checksum for a sentence (without $ and *)."""
    checksum = 0
    for char in sentence:
        checksum ^= ord(char)
    return f"{checksum:02X}"

def generate_gpgga():
    """Generate a $GPGGA sentence with approximate location."""
    # Format: $GPGGA,hhmmss.ss,ddmm.mmmmm,s,dddmm.mmmmm,s,q,ss,h.h,a.a,M,g.g,M,a.a,xxxx*hh
    utc_time = time.strftime("%H%M%S.00", time.gmtime())  # Current UTC time
    sentence = (
        f"GPGGA,{utc_time},{LATITUDE},{LONGITUDE},1,12,1.0,{ALTITUDE},M,0.0,M,,"
    )
    checksum = calculate_checksum(sentence)
    return f"${sentence}*{checksum}\r\n"

def parse_gpgga(sentence):
    """Parse a $GPGGA NMEA sentence and extract RTK status."""
    try:
        parts = sentence.strip().split(',')
        if parts[0] != "$GPGGA" or len(parts) < 10:
            return

        fix_type = int(parts[6]) if parts[6] else 0
        satellites = int(parts[7]) if parts[7] else 0
        hdop = float(parts[8]) if parts[8] else None
        altitude = float(parts[9]) if parts[9] else None

        latest_gps_data["fix_type"] = fix_type
        latest_gps_data["satellites"] = satellites
        latest_gps_data["hdop"] = hdop
        latest_gps_data["altitude"] = altitude

    except (ValueError, IndexError) as e:
        print(f"Error parsing GPGGA sentence: {e}")

def report_rtk_status():
    """Report RTK status every 5 seconds."""
    while True:
        time.sleep(5)
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
            line = gps_serial.readline().decode('ascii', errors='ignore').strip()
            if line.startswith("$GPGGA"):
                parse_gpgga(line)
        except Exception as e:
            print(f"Error reading NMEA: {e}")
            time.sleep(1)

def fetch_rtcm():
    """Fetch RTCM data from NTRIP server and send to GPS, while reading NMEA."""
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.settimeout(10)

    try:
        print(f"Attempting to connect to {NTRIP_SERVER}:{NTRIP_PORT}...")
        client.connect((NTRIP_SERVER, NTRIP_PORT))
        print(f"Connected to {NTRIP_SERVER}:{NTRIP_PORT}")

        auth = base64.b64encode(f"{USERNAME}:{PASSWORD}".encode()).decode()
        gpgga = generate_gpgga()
        headers = (
            f"GET /{MOUNTPOINT} HTTP/1.1\r\n"
            f"Host: {NTRIP_SERVER}:{NTRIP_PORT}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"User-Agent: NTRIP RTKLIB/2.4.3\r\n"
            f"Accept: */*\r\n"
            f"Connection: close\r\n"
            f"Authorization: Basic {auth}\r\n"
            f"Ntrip-GGA: {gpgga}\r\n"
            f"\r\n"
        )
        print("Sending NTRIP request headers:")
        print(headers)
        client.send(headers.encode())

        response = client.recv(1024).decode()
        print("Server response:", response)
        if "ICY 200 OK" not in response and "HTTP/1.1 200 OK" not in response:
            print("Failed to connect to NTRIP server")
            raise Exception("NTRIP connection failed")

        print("NTRIP server connected successfully")

        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as gps_serial:
            print(f"Opened serial port {SERIAL_PORT} at {BAUDRATE} baud")

            nmea_thread = threading.Thread(target=read_nmea, args=(gps_serial,), daemon=True)
            nmea_thread.start()

            report_thread = threading.Thread(target=report_rtk_status, daemon=True)
            report_thread.start()

            last_gga_time = time.time()
            while True:
                data = client.recv(1024)
                if not data:
                    print("No more data from NTRIP server")
                    break

                gps_serial.write(data)
                print(f"Sent {len(data)} bytes to GPS")

                # Send $GPGGA every 10 seconds (per RTCM recommendation)
                current_time = time.time()
                if current_time - last_gga_time >= 10:
                    gpgga = generate_gpgga()
                    gps_serial.write(gpgga.encode())
                    print(f"Sent $GPGGA: {gpgga.strip()}")
                    last_gga_time = current_time

                time.sleep(0.01)

    except Exception as e:
        print(f"Error: {e}")
        print(traceback.format_exc())
    finally:
        client.close()
        print("NTRIP connection closed")

if __name__ == "__main__":
    fetch_rtcm()