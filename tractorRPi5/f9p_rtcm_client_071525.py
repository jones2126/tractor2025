import serial
import socket
import threading
import re
import time
import csv
from datetime import datetime

# --- Configuration ---
TCP_IP = '192.168.1.233'
TCP_PORT = 6001
# SERIAL_PORT = 'COM41'  # Change to '/dev/ttyUSB0' on Raspberry Pi
SERIAL_PORT = '/dev/ttyACM1'  # Raspberry Pi
SERIAL_BAUDRATE = 115200
RTCM_TIMEOUT = 5  # seconds without RTCM = warning
CSV_LOGFILE = 'fix_status_log.csv'

# Regex to extract GNGGA fix quality
GGA_PATTERN = re.compile(rb'\$GNGGA,[^,]*,[^,]*,[^,]*,[^,]*,[^,]*,(\d),')
FIX_QUALITY = {
    0: 'Invalid',
    1: 'GPS Fix',
    2: 'DGPS',
    4: 'RTK Fixed',
    5: 'RTK Float'
}

# Shared state
last_rtcm_time = time.time()
csv_lock = threading.Lock()

def forward_rtcm_data(tcp_sock, serial_conn):
    """Stream RTCM from TCP to serial (GNSS) and update last RTCM time."""
    global last_rtcm_time
    while True:
        try:
            data = tcp_sock.recv(1024)
            if not data:
                print("[RTCM] Stream ended.")
                break
            total_sent = 0
            while total_sent < len(data):
                sent = serial_conn.write(data[total_sent:])
                total_sent += sent
            last_rtcm_time = time.time()
        except Exception as e:
            print(f"[RTCM Error] {e}")
            break

def log_to_csv(timestamp, fix_code, status):
    """Append fix transition to CSV log file."""
    with csv_lock:
        with open(CSV_LOGFILE, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([timestamp, fix_code, status])

def monitor_gngga(serial_conn):
    """Parse GNGGA sentences for fix quality and log transitions."""
    buffer = b""
    last_fix_code = None
    while True:
        try:
            byte = serial_conn.read(1)
            if not byte:
                continue
            buffer += byte
            if byte == b'\n':
                if b'GNGGA' in buffer:
                    match = GGA_PATTERN.search(buffer)
                    if match:
                        fix_code = int(match.group(1))
                        if fix_code != last_fix_code:
                            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                            status = FIX_QUALITY.get(fix_code, f"Unknown ({fix_code})")
                            print(f"[{timestamp}] Fix Quality: {fix_code} → {status}")
                            log_to_csv(timestamp, fix_code, status)
                            last_fix_code = fix_code
                buffer = b""
        except Exception as e:
            print(f"[GNGGA Error] {e}")
            break

def monitor_rtcm_health():
    """Print warning if no RTCM data has arrived in the last 5 seconds."""
    while True:
        time.sleep(1)
        if time.time() - last_rtcm_time > RTCM_TIMEOUT:
            print(f"[WARNING] No RTCM received for > {RTCM_TIMEOUT} seconds.")

def main():
    try:
        print(f"[INFO] Connecting to RTCM stream at {TCP_IP}:{TCP_PORT}...")
        tcp_sock = socket.create_connection((TCP_IP, TCP_PORT), timeout=10)
        print("[INFO] RTCM TCP connection established.")

        print(f"[INFO] Opening serial port {SERIAL_PORT} at {SERIAL_BAUDRATE}...")
        serial_conn = serial.Serial(
            port=SERIAL_PORT,
            baudrate=SERIAL_BAUDRATE,
            timeout=1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            write_timeout=1,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )
        print("[INFO] Serial port opened.")

        # Write CSV header
        with open(CSV_LOGFILE, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Timestamp', 'Fix Code', 'Fix Status'])

        # Start threads
        threading.Thread(target=forward_rtcm_data, args=(tcp_sock, serial_conn), daemon=True).start()
        threading.Thread(target=monitor_gngga, args=(serial_conn,), daemon=True).start()
        threading.Thread(target=monitor_rtcm_health, daemon=True).start()

        while True:
            time.sleep(1)

    except Exception as e:
        print(f"[Startup Error] {e}")

if __name__ == "__main__":
    main()
