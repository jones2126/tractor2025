import serial
import time
import logging
from pyubx2 import UBXMessage

# --------------------------
# CONFIGURATION
# --------------------------
PORT = "/dev/f9p"   # Change to match your port
BAUDRATE = 115200
MONITOR_TIME = 5  # seconds to monitor NMEA output after reset

# --------------------------
# LOGGING SETUP
# --------------------------
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

def factory_reset(ser):
    """
    Send CFG-CFG command with clearMask, loadMask, storeMask set to reset device.
    """
    logging.info("Sending factory reset...")
    msg = UBXMessage(
        "CFG",
        "CFG-CFG",
        msgmode=0,  # SET mode
        clearMask=b"\xFF\xFF\x00\x00",  # Clear all configurations
        saveMask=b"\x00\x00\x00\x00",   # Don't save
        loadMask=b"\xFF\xFF\x00\x00",   # Reload defaults
        devBBR=1,
        devFlash=1,
        devEEPROM=1
    )
    ser.write(msg.serialize())
    ser.flush()

def monitor_nmea(ser):
    """
    Read raw serial data and collect all unique NMEA sentence identifiers for MONITOR_TIME seconds.
    """
    logging.info(f"Monitoring NMEA output for {MONITOR_TIME}s...")
    nmea_sentences = set()
    ser.timeout = 0.5  # prevent hanging on readline
    start_time = time.time()

    while time.time() - start_time < MONITOR_TIME:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if line.startswith("$"):
                # Example: $GPGSV -> GSV
                parts = line.split(",")[0]
                talker = parts[1:3]   # GP, GN, GA, etc.
                sentence = parts[3:]  # GSV, GGA, etc.
                nmea_sentences.add(f"{talker}{sentence}")
        except Exception:
            pass

    if nmea_sentences:
        logging.info(f"NMEA sentences detected: {', '.join(sorted(nmea_sentences))}")
    else:
        logging.info("No NMEA messages detected in monitoring window.")

def main():
    logging.info(f"Connecting to F9P on {PORT} @ {BAUDRATE}")
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
        factory_reset(ser)
        logging.info("Waiting 1s for receiver to reboot...")
        time.sleep(1)
        monitor_nmea(ser)

if __name__ == "__main__":
    main()
