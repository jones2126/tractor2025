"""
dual_gps_rtk_client.py
=====================
This script runs on a Raspberry Pi 5 to connect to a TCP server on a Raspberry Pi 3 (192.168.1.233:6001),
receive RTCM data, and forward it to TWO rovers:
- Base link GPS (ZED-F9P on /dev/ttyACM1) - mounted above rear axle center
- Heading GPS (ZED-F9P on /dev/ttyACM2) - mounted ~1m forward of base link

It monitors both GPS units' NMEA output to check RTK status, calculates heading from the two GPS positions,
and publishes navigation data via Unix socket for the navigation system.
"""

import socket
import serial
import threading
import time
from datetime import datetime
import os
import logging
import logging.handlers
import struct
import glob
import gzip
import math
import json

# Configure application logging
logger = logging.getLogger('DualGPSRTCMClient')
logger.setLevel(logging.INFO)
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')

log_dir = "logs"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

app_log_handler = logging.handlers.TimedRotatingFileHandler(
    filename=os.path.join(log_dir, 'dual_gps_rtcm_client.log'),
    when='midnight',
    interval=1,
    backupCount=7
)
app_log_handler.setFormatter(formatter)
logger.addHandler(app_log_handler)
console_handler = logging.StreamHandler()
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)

# Configuration
TCP_HOST = "192.168.1.233"
TCP_PORT = 6001
BASE_LINK_PORT = "/dev/ttyACM1"    # Base link GPS (rear axle center)
HEADING_PORT = "/dev/ttyACM2"      # Heading GPS (~1m forward)
BAUD_RATE = 115200
STATUS_INTERVAL = 5
RATE_REPORT_INTERVAL = 15
LOG_INTERVAL = 3600
LOG_RETENTION = 24
RECONNECT_DELAY = 5
COMPRESS_LOGS = True

# Navigation data socket
NAV_SOCKET_PATH = "/tmp/gps_nav_data.sock"

# GPS data storage
class GPSData:
    def __init__(self, name):
        self.name = name
        self.lat = None
        self.lon = None
        self.altitude = None
        self.rtk_status = "No Fix"
        self.rtk_status_code = "0"
        self.num_satellites = 0
        self.last_update = None
        self.first_rtk_time = None
        
    def update_from_gga(self, gga_line):
        """Update GPS data from GGA sentence"""
        try:
            fields = gga_line.split(',')
            if len(fields) < 15:
                return False
                
            # Parse latitude
            if fields[2] and fields[3]:
                lat_deg = float(fields[2][:2])
                lat_min = float(fields[2][2:])
                self.lat = lat_deg + lat_min/60.0
                if fields[3] == 'S':
                    self.lat = -self.lat
                    
            # Parse longitude  
            if fields[4] and fields[5]:
                lon_deg = float(fields[4][:3])
                lon_min = float(fields[4][3:])
                self.lon = lon_deg + lon_min/60.0
                if fields[5] == 'W':
                    self.lon = -self.lon
                    
            # Parse other fields
            self.rtk_status_code = fields[6] if fields[6] else "0"
            self.num_satellites = int(fields[7]) if fields[7] else 0
            self.altitude = float(fields[9]) if fields[9] else None
            
            # Map status codes to descriptions
            status_map = {
                "0": "No Fix",
                "1": "GPS Fix", 
                "2": "DGPS Fix",
                "4": "RTK Fixed",
                "5": "RTK Float"
            }
            self.rtk_status = status_map.get(self.rtk_status_code, "Unknown")
            
            # Track first RTK fix
            if self.rtk_status_code in ["4", "5"] and self.first_rtk_time is None:
                self.first_rtk_time = time.time()
                logger.info(f"{self.name}: First RTK fix achieved!")
                
            self.last_update = time.time()
            return True
            
        except Exception as e:
            logger.error(f"Error parsing GGA for {self.name}: {e}")
            return False

def calculate_rtcm_crc(data):
    """Calculate the 24-bit CRC for an RTCM message."""
    crc = 0
    for i in range(len(data)):
        crc ^= (data[i] << 16)
        for j in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
        crc &= 0xFFFFFF
    return crc

def parse_rtcm_message(data, pos, crc_errors):
    """Parse an RTCM message starting at pos."""
    if len(data) < pos + 3:
        return None, None, pos, None
    if data[pos] != 0xD3:
        return None, None, pos, None

    length = ((data[pos + 1] & 0x03) << 8) | data[pos + 2]
    total_length = 3 + length + 3
    if len(data) < pos + total_length:
        return None, None, pos, None

    message = data[pos:pos + total_length]
    calculated_crc = calculate_rtcm_crc(message[:-3])
    crc_bytes = message[-3:]
    received_crc = struct.unpack('>I', b'\x00' + crc_bytes)[0]
    if calculated_crc != received_crc:
        return None, None, pos + 1, None

    if len(message) < 5:
        return None, None, pos + 1, None
    msg_type = struct.unpack('>H', message[3:5])[0] >> 4
    new_pos = pos + total_length

    return msg_type, length, new_pos, None

def calculate_heading(base_lat, base_lon, heading_lat, heading_lon):
    """
    Calculate heading from base_link GPS to heading GPS.
    Returns heading in degrees (0-360, where 0 is North).
    """
    if None in [base_lat, base_lon, heading_lat, heading_lon]:
        return None
        
    # Convert to radians
    lat1 = math.radians(base_lat)
    lon1 = math.radians(base_lon)
    lat2 = math.radians(heading_lat)
    lon2 = math.radians(heading_lon)
    
    # Calculate difference in longitude
    dlon = lon2 - lon1
    
    # Calculate bearing
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    
    bearing = math.atan2(y, x)
    
    # Convert to degrees and normalize to 0-360
    bearing_deg = math.degrees(bearing)
    heading = (bearing_deg + 360) % 360
    
    return heading

def connect_to_server():
    """Connect to the TCP server with retry logic."""
    while True:
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((TCP_HOST, TCP_PORT))
            logger.info(f"Connected to TCP server at {TCP_HOST}:{TCP_PORT}")
            return client_socket
        except Exception as e:
            logger.error(f"Failed to connect to TCP server: {e}. Retrying in {RECONNECT_DELAY} seconds...")
            time.sleep(RECONNECT_DELAY)

def setup_nav_socket():
    """Setup Unix socket server for navigation data."""
    # Remove existing socket if it exists
    try:
        os.unlink(NAV_SOCKET_PATH)
    except OSError:
        pass
        
    nav_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    nav_socket.bind(NAV_SOCKET_PATH)
    nav_socket.listen(5)
    logger.info(f"Navigation data socket listening on {NAV_SOCKET_PATH}")
    return nav_socket

def broadcast_nav_data(nav_clients, base_gps, heading_gps, calculated_heading):
    """Broadcast navigation data to connected clients."""
    if not nav_clients:
        return
        
    # Create navigation message
    nav_msg = {
        "header": "gps",
        "timestamp": time.time(),
        "base_link": {
            "lat": base_gps.lat,
            "lon": base_gps.lon, 
            "altitude": base_gps.altitude,
            "rtk_status": base_gps.rtk_status,
            "rtk_status_code": base_gps.rtk_status_code,
            "satellites": base_gps.num_satellites
        },
        "heading_gps": {
            "lat": heading_gps.lat,
            "lon": heading_gps.lon,
            "altitude": heading_gps.altitude, 
            "rtk_status": heading_gps.rtk_status,
            "rtk_status_code": heading_gps.rtk_status_code,
            "satellites": heading_gps.num_satellites
        },
        "calculated_heading": calculated_heading
    }
    
    # Convert to comma-delimited string
    csv_msg = f"gps,{nav_msg['timestamp']},{base_gps.lat or 0},{base_gps.lon or 0}," \
              f"{base_gps.altitude or 0},{base_gps.rtk_status_code}," \
              f"{calculated_heading or 0},{base_gps.num_satellites}\n"
    
    # Send to all connected clients
    disconnected_clients = []
    for client in nav_clients:
        try:
            client.send(csv_msg.encode())
        except:
            disconnected_clients.append(client)
            
    # Remove disconnected clients
    for client in disconnected_clients:
        nav_clients.remove(client)
        try:
            client.close()
        except:
            pass

def handle_nav_connections(nav_socket, nav_clients):
    """Handle incoming navigation socket connections."""
    while True:
        try:
            client_conn, client_addr = nav_socket.accept()
            nav_clients.append(client_conn)
            logger.info(f"Navigation client connected")
        except Exception as e:
            logger.error(f"Error accepting navigation connection: {e}")
            time.sleep(1)

# Shared state for RTCM monitoring
last_rtcm_time = time.time()

def forward_rtcm_data_with_reconnect(base_serial, heading_serial):
    """Forward RTCM data to both GPS units with robust TCP reconnection."""
    global last_rtcm_time
    rtcm_buffer = b''
    message_counts = {}
    crc_errors = 0
    last_rate_check = time.time()
    
    while True:
        try:
            logger.info(f"Connecting to RTCM stream at {TCP_HOST}:{TCP_PORT}...")
            tcp_sock = socket.create_connection((TCP_HOST, TCP_PORT), timeout=10)
            logger.info("RTCM TCP connection established.")

            while True:
                data = tcp_sock.recv(2048)
                if not data:
                    raise ConnectionError("No data from RTCM stream (connection closed)")
                
                rtcm_buffer += data
                
                # Parse RTCM messages
                pos = 0
                valid_rtcm_data = b''
                while pos < len(rtcm_buffer):
                    message_type, length, new_pos, decoded_data = parse_rtcm_message(rtcm_buffer, pos, crc_errors)
                    if message_type is None:
                        if new_pos > pos:
                            crc_errors += 1
                        pos = new_pos if new_pos > pos else pos + 1
                        continue
                    
                    # Extract valid message
                    message_length = 3 + length + 3
                    valid_rtcm_data += rtcm_buffer[pos:pos + message_length]
                    message_counts[message_type] = message_counts.get(message_type, 0) + 1
                    pos = new_pos

                # Forward RTCM data to both GPS units
                if valid_rtcm_data:
                    try:
                        total_sent_base = 0
                        while total_sent_base < len(valid_rtcm_data):
                            sent = base_serial.write(valid_rtcm_data[total_sent_base:])
                            total_sent_base += sent
                        base_serial.flush()
                        
                        total_sent_heading = 0
                        while total_sent_heading < len(valid_rtcm_data):
                            sent = heading_serial.write(valid_rtcm_data[total_sent_heading:])
                            total_sent_heading += sent
                        heading_serial.flush()
                        
                        last_rtcm_time = time.time()
                    except Exception as e:
                        logger.error(f"Error forwarding RTCM data: {e}")

                # Remove processed messages
                rtcm_buffer = rtcm_buffer[pos:]
                
                # Report RTCM message rates
                current_time = time.time()
                if current_time - last_rate_check >= RATE_REPORT_INTERVAL:
                    elapsed = current_time - last_rate_check
                    if elapsed > 0:
                        logger.info("RTCM Message Rates (messages/sec):")
                        total_count = 0
                        for msg_type, count in sorted(message_counts.items()):
                            rate = count / elapsed
                            total_count += count
                            logger.info(f"  Type {msg_type}: {rate:.2f} Hz ({count} messages)")
                        total_rate = total_count / elapsed
                        logger.info(f"  Total: {total_rate:.2f} Hz ({total_count} messages)")
                    message_counts.clear()
                    last_rate_check = current_time

        except Exception as e:
            logger.error(f"[RTCM Error] {e}")
            logger.info("Will retry RTCM connection in 30 seconds...")
            time.sleep(30)

def monitor_gps_nmea(gps_data, serial_conn):
    """Monitor NMEA data from a GPS unit using byte-by-byte reading like your working script."""
    buffer = b""
    last_fix_code = None
    
    while True:
        try:
            byte = serial_conn.read(1)
            if not byte:
                continue
            buffer += byte
            if byte == b'\n':
                try:
                    line = buffer.decode('utf-8', errors='ignore').strip()
                    if line.startswith("$GPGGA") or line.startswith("$GNGGA"):
                        success = gps_data.update_from_gga(line)
                        if success:
                            # Log RTK status changes like your working script
                            current_fix = gps_data.rtk_status_code
                            if current_fix != last_fix_code:
                                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                                logger.info(f"[{timestamp}] {gps_data.name} Fix Quality: {current_fix} → {gps_data.rtk_status}")
                                last_fix_code = current_fix
                except Exception as e:
                    logger.error(f"Error decoding NMEA for {gps_data.name}: {e}")
                buffer = b""
        except Exception as e:
            logger.error(f"[{gps_data.name} NMEA Error] {e}")
            time.sleep(1)

def monitor_rtcm_health():
    """Monitor RTCM health and warn if no data received."""
    while True:
        time.sleep(1)
        elapsed = time.time() - last_rtcm_time
        if elapsed > 5.0:  # 5 second timeout
            logger.warning(f"No RTCM received for {elapsed:.1f} seconds.")

def status_reporter(base_gps, heading_gps, nav_clients):
    """Separate thread for status reporting and navigation data broadcasting."""
    last_status_time = time.time()
    last_nav_broadcast = time.time()
    
    while True:
        current_time = time.time()
        
        # Report status periodically
        if current_time - last_status_time >= STATUS_INTERVAL:
            logger.info(f"Base Link: {base_gps.rtk_status} | Sats: {base_gps.num_satellites}")
            logger.info(f"Heading: {heading_gps.rtk_status} | Sats: {heading_gps.num_satellites}")
            
            # Calculate and log heading if both GPS have fixes
            calculated_heading = calculate_heading(
                base_gps.lat, base_gps.lon,
                heading_gps.lat, heading_gps.lon
            )
            if calculated_heading is not None:
                logger.info(f"Calculated Heading: {calculated_heading:.2f}°")
            
            last_status_time = current_time
        
        # Broadcast navigation data at 5Hz
        if current_time - last_nav_broadcast >= 0.2:  # 5Hz
            calculated_heading = calculate_heading(
                base_gps.lat, base_gps.lon,
                heading_gps.lat, heading_gps.lon
            )
            broadcast_nav_data(nav_clients, base_gps, heading_gps, calculated_heading)
            last_nav_broadcast = current_time
            
        time.sleep(0.1)

def main():
    """Main function to handle dual GPS RTK client with threading architecture."""
    # Initialize GPS data objects
    base_link_gps = GPSData("Base Link")
    heading_gps = GPSData("Heading")
    
    # Setup navigation socket
    nav_socket = setup_nav_socket()
    nav_clients = []
    
    # Initialize serial ports
    try:
        logger.info(f"Opening serial port {BASE_LINK_PORT} for Base Link GPS...")
        base_serial = serial.Serial(
            port=BASE_LINK_PORT,
            baudrate=BAUD_RATE,
            timeout=1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            write_timeout=1,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )
        logger.info("Base Link GPS serial port opened.")
        
        logger.info(f"Opening serial port {HEADING_PORT} for Heading GPS...")
        heading_serial = serial.Serial(
            port=HEADING_PORT,
            baudrate=BAUD_RATE,
            timeout=1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            write_timeout=1,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )
        logger.info("Heading GPS serial port opened.")
        
    except serial.SerialException as e:
        logger.error(f"Failed to open GPS serial ports: {e}")
        return
    
    # Start all threads
    logger.info("Starting threads...")
    
    # Navigation connection handler
    threading.Thread(target=handle_nav_connections, args=(nav_socket, nav_clients), daemon=True).start()
    
    # RTCM forwarding with reconnection
    threading.Thread(target=forward_rtcm_data_with_reconnect, args=(base_serial, heading_serial), daemon=True).start()
    
    # GPS NMEA monitoring
    threading.Thread(target=monitor_gps_nmea, args=(base_link_gps, base_serial), daemon=True).start()
    threading.Thread(target=monitor_gps_nmea, args=(heading_gps, heading_serial), daemon=True).start()
    
    # RTCM health monitoring
    threading.Thread(target=monitor_rtcm_health, daemon=True).start()
    
    # Status reporting and navigation broadcasting
    threading.Thread(target=status_reporter, args=(base_link_gps, heading_gps, nav_clients), daemon=True).start()
    
    logger.info("All threads started. Main loop running...")
    
    # Main thread just keeps the program alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        # Cleanup
        try:
            base_serial.close()
            heading_serial.close()
            nav_socket.close()
            os.unlink(NAV_SOCKET_PATH)
        except:
            pass

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger.info("Exiting...")
        try:
            os.unlink(NAV_SOCKET_PATH)
        except OSError:
            pass
