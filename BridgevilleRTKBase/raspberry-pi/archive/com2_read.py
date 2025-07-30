import serial
import time

# Serial port settings for COM2
port = "/dev/ttyUSB1"  # COM2 on your Raspberry Pi
baudrate = 115200
timeout = 1  # Timeout for reading (in seconds)

def parse_gpgga(sentence):
    """Parse a GPGGA NMEA sentence and return key fields."""
    if not sentence.startswith("$GPGGA"):
        return None

    fields = sentence.split(",")
    if len(fields) < 15:
        return None

    try:
        # Extract key fields
        time_utc = fields[1]  # UTC time (HHMMSS.ss)
        latitude = fields[2]  # Latitude (DDMM.MMMM)
        lat_dir = fields[3]   # Latitude direction (N/S)
        longitude = fields[4] # Longitude (DDDMM.MMMM)
        lon_dir = fields[5]   # Longitude direction (E/W)
        fix_quality = fields[6]  # Fix quality (0=invalid, 1=GPS fix, etc.)
        num_satellites = fields[7]  # Number of satellites
        hdop = fields[8]  # Horizontal dilution of precision
        altitude = fields[9]  # Altitude above mean sea level (meters)
        checksum = fields[14].split("*")[1]  # Checksum

        # Convert latitude and longitude to decimal degrees
        lat_deg = float(latitude[:2]) + float(latitude[2:]) / 60
        if lat_dir == "S":
            lat_deg = -lat_deg
        lon_deg = float(longitude[:3]) + float(longitude[3:]) / 60
        if lon_dir == "W":
            lon_deg = -lon_deg

        return {
            "time_utc": time_utc,
            "latitude": lat_deg,
            "longitude": lon_deg,
            "fix_quality": fix_quality,
            "num_satellites": num_satellites,
            "hdop": hdop,
            "altitude": altitude,
            "checksum": checksum
        }
    except (IndexError, ValueError):
        return None

try:
    # Open the serial port
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
        xonxoff=False,  # Disable software flow control
        rtscts=False,   # Disable hardware flow control (RTS/CTS)
        dsrdtr=False    # Disable hardware flow control (DSR/DTR)
    )

    print(f"Connected to {port} at {baudrate} baud")

    # Ensure the serial port is open
    if ser.is_open:
        print("Reading NMEA messages from COM2... (press Ctrl+C to stop)")
        try:
            while True:
                # Read a line from COM2
                if ser.in_waiting > 0:
                    line = ser.readline().decode('ascii', errors='ignore').strip()
                    if line:
                        print(f"Raw NMEA: {line}")
                        # Parse if it's a GPGGA sentence
                        parsed = parse_gpgga(line)
                        if parsed:
                            print("Parsed GPGGA:")
                            print(f"  Time (UTC): {parsed['time_utc']}")
                            print(f"  Latitude: {parsed['latitude']:.6f}°")
                            print(f"  Longitude: {parsed['longitude']:.6f}°")
                            print(f"  Fix Quality: {parsed['fix_quality']}")
                            print(f"  Satellites: {parsed['num_satellites']}")
                            print(f"  HDOP: {parsed['hdop']}")
                            print(f"  Altitude: {parsed['altitude']} m")
                            print(f"  Checksum: {parsed['checksum']}")
                            print("---")
                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\nStopped by user")

    else:
        print("Failed to open serial port")

except serial.SerialException as e:
    print(f"Serial error: {e}")
except Exception as e:
    print(f"Error: {e}")
finally:
    # Close the serial port
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed")
