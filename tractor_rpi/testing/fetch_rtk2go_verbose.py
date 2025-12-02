import socket
import base64
import serial
import time
import threading
import traceback
import sys

# NTRIP server details - use the hostname instead of IP
NTRIP_SERVER = "rtk2go.com"
NTRIP_PORT = 2101
MOUNTPOINT = "cmuairlab01"
USERNAME = "aej2126l@protonmail.com"
PASSWORD = "none"

# Serial port details for Raspberry Pi
SERIAL_PORT = "/dev/ttyUSB0"  # Common for USB-to-Serial adapters on Linux
BAUDRATE = 115200

# Global variables for RTK status
latest_gps_data = {
    "fix_type": None,
    "satellites": None,
    "hdop": None,
    "altitude": None,
    "last_report_time": 0
}

def read_nmea(gps_serial):
    """Read NMEA sentences from the GPS and parse them."""
    while True:
        try:
            line = gps_serial.readline().decode('ascii', errors='ignore').strip()
            if line.startswith("$GPGGA"):
                # Just log the received GPGGA message
                print(f"Received NMEA: {line}")
        except Exception as e:
            print(f"Error reading NMEA: {e}")
            time.sleep(1)

def report_rtk_status():
    """Report RTK status every 5 seconds."""
    while True:
        time.sleep(5)
        # Just report that we're receiving RTCM data
        print("RTK status: Receiving RTCM corrections data")

def check_source_table():
    """Get the NTRIP source table to verify mountpoint."""
    try:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(10)
        print(f"Attempting to connect to {NTRIP_SERVER}:{NTRIP_PORT} for source table...")
        client.connect((NTRIP_SERVER, NTRIP_PORT))
        print(f"Connected to {NTRIP_SERVER}:{NTRIP_PORT}")
        
        # Prepare authentication
        auth_string = f"{USERNAME}:{PASSWORD}"
        auth = base64.b64encode(auth_string.encode()).decode()
        
        # Request source table
        headers = (
            f"GET / HTTP/1.0\r\n"
            f"User-Agent: NTRIP GNSSViewer/1.0\r\n"
            f"Authorization: Basic {auth}\r\n"
            f"\r\n"
        )
        
        client.send(headers.encode())
        print("Sent source table request")
        
        # Read the response but don't print it
        response = b""
        while True:
            try:
                chunk = client.recv(4096)
                if not chunk:
                    break
                response += chunk
            except socket.timeout:
                break
        
        # Check if our mountpoint exists (without printing the whole table)
        response_text = response.decode('utf-8', errors='ignore')
        
        if MOUNTPOINT in response_text:
            print(f"✓ Mountpoint '{MOUNTPOINT}' found in source table!")
            print("Source table request successful.")
            return True
        else:
            print(f"❌ Mountpoint '{MOUNTPOINT}' not found in the source table.")
            print("Please check if you're using the correct mountpoint name.")
            return False
        
    except Exception as e:
        print(f"Error checking source table: {e}")
        return False
    finally:
        try:
            client.close()
        except:
            pass

def fetch_rtcm():
    """Fetch RTCM data from NTRIP server and send to GPS."""
    # Check source table first
    print("Verifying mountpoint availability...")
    check_source_table()
        
    max_retries = 3
    retry_count = 0
    
    while retry_count < max_retries:
        client = None
        try:
            print("Waiting 2 seconds before connecting to avoid rapid reconnections...")
            time.sleep(2)
            
            # Create socket
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.settimeout(10)
            
            print(f"Attempting to connect to {NTRIP_SERVER}:{NTRIP_PORT}...")
            client.connect((NTRIP_SERVER, NTRIP_PORT))
            print(f"Connected to {NTRIP_SERVER}:{NTRIP_PORT}")

            # Prepare authentication
            auth_string = f"{USERNAME}:{PASSWORD}"
            auth = base64.b64encode(auth_string.encode()).decode()
            
            # Use NTRIP v1 protocol (most compatible)
            print("Using NTRIP v1 protocol...")
            headers = (
                f"GET /{MOUNTPOINT} HTTP/1.0\r\n"
                f"User-Agent: NTRIP GNSSViewer/1.0\r\n"
                f"Authorization: Basic {auth}\r\n"
                f"\r\n"
            )
            
            print("Sending NTRIP request headers...")
            client.send(headers.encode())

            # Receive and process initial response (just the header)
            response_buffer = b""
            while b"\r\n\r\n" not in response_buffer and len(response_buffer) < 4096:
                try:
                    chunk = client.recv(1024)
                    if not chunk:
                        break
                    response_buffer += chunk
                except socket.timeout:
                    break
            
            # Split header from any data that might be included
            header_end = response_buffer.find(b"\r\n\r\n")
            if header_end >= 0:
                header = response_buffer[:header_end+4]
                initial_data = response_buffer[header_end+4:]
                header_text = header.decode('utf-8', errors='ignore')
                print("Server response:", header_text)
            else:
                header_text = response_buffer.decode('utf-8', errors='ignore')
                print("Server response:", header_text)
                initial_data = b""
            
            # Check for successful response
            if "200 OK" not in header_text:
                print("Failed to connect to NTRIP server")
                print(f"Response: {header_text}")
                
                # Provide specific error feedback
                if "401" in header_text:
                    print("Error: Authentication failed. Check username and password.")
                elif "403" in header_text:
                    print("Error: Access forbidden. No permission for this mountpoint.")
                elif "404" in header_text:
                    print("Error: Mountpoint not found. Check mountpoint name.")
                else:
                    print("Error: Unknown error. Check connection settings.")
                
                retry_count += 1
                if retry_count < max_retries:
                    print(f"Retrying ({retry_count}/{max_retries})...")
                    time.sleep(5)
                continue
            
            print("NTRIP server connected successfully")

            # Sample GPGGA message - replace with your actual location
            gpgga = "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*76\r\n"
            
            # Open serial port
            try:
                gps_serial = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
                print(f"Opened serial port {SERIAL_PORT} at {BAUDRATE} baud")
                
                # Start NMEA reading in a separate thread
                nmea_thread = threading.Thread(target=read_nmea, args=(gps_serial,), daemon=True)
                nmea_thread.start()

                # Start status reporting in a separate thread
                report_thread = threading.Thread(target=report_rtk_status, daemon=True)
                report_thread.start()
                
                # Send initial data if any
                if initial_data:
                    gps_serial.write(initial_data)
                    print(f"Sent {len(initial_data)} bytes of RTCM data to GPS")
            except Exception as e:
                print(f"Error opening serial port: {e}")
                print("Please check your GPS connection and port settings.")
                print("If using a USB-to-Serial adapter, try /dev/ttyUSB0 or /dev/ttyACM0")
                print("You may need to run this script with sudo for port access")
                break
            
            # Send initial position to NTRIP server
            client.send(gpgga.encode())
            print(f"Sent initial position data to NTRIP server")
            
            last_gpgga_time = time.time()
            data_received = 0
            
            # Main data receiving loop
            while True:
                # Send GPGGA update every 10 seconds
                current_time = time.time()
                if current_time - last_gpgga_time >= 10:
                    client.send(gpgga.encode())
                    print(f"Sent position update to NTRIP server")
                    last_gpgga_time = current_time
                
                # Receive RTCM data with a short timeout
                client.settimeout(1.0)
                try:
                    data = client.recv(1024)
                    if not data:
                        print("Connection closed by server")
                        break
                    
                    # Send to GPS and count bytes
                    gps_serial.write(data)
                    data_received += len(data)
                    print(f"Sent {len(data)} bytes of RTCM data to GPS (Total: {data_received} bytes)")
                    
                except socket.timeout:
                    # Timeout is normal, just continue
                    continue
                except Exception as e:
                    print(f"Error receiving data: {e}")
                    break
            
            # Close serial port and socket properly
            try:
                gps_serial.close()
                print("Serial port closed")
            except:
                pass
            
            break  # Exit retry loop if successful

        except Exception as e:
            print(f"Error: {e}")
            retry_count += 1
            if retry_count < max_retries:
                print(f"Retrying ({retry_count}/{max_retries})...")
                time.sleep(5)
        finally:
            if client:
                try:
                    client.close()
                except:
                    pass
            print("NTRIP connection closed")
    
    if retry_count >= max_retries:
        print(f"Failed after {max_retries} attempts. Check your connection settings.")

def list_serial_ports():
    """List available serial ports on Raspberry Pi/Linux."""
    import os
    
    # Common USB-to-Serial devices on Linux
    usb_serial_patterns = [
        '/dev/ttyUSB*',     # Most USB-to-Serial adapters
        '/dev/ttyACM*',     # Arduino and some GPS devices
        '/dev/ttyS*',       # Hardware serial ports
        '/dev/serial/by-id/*'  # Persistent device names
    ]
    
    found_ports = []
    
    for pattern in usb_serial_patterns:
        import glob
        matches = glob.glob(pattern)
        for port in matches:
            if os.path.exists(port):
                found_ports.append(port)
    
    if found_ports:
        print("Found serial ports:")
        for port in found_ports:
            print(f"  - {port}")
    else:
        print("No serial ports found. If using a USB adapter, make sure it's connected.")
        print("You might need to run this script with sudo for port access.")
    
    return found_ports

if __name__ == "__main__":
    try:
        if len(sys.argv) > 1:
            if sys.argv[1] == "--ports":
                list_serial_ports()
            elif sys.argv[1].startswith("/dev/"):
                SERIAL_PORT = sys.argv[1]
                print(f"Using specified serial port: {SERIAL_PORT}")
                fetch_rtcm()
            else:
                print("Unknown command. Available commands:")
                print("  --ports  : List available serial ports")
                print("  /dev/X   : Use specific serial port (e.g., /dev/ttyUSB0)")
        else:
            fetch_rtcm()
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    except Exception as e:
        print(f"Unhandled exception: {e}")
        traceback.print_exc()
