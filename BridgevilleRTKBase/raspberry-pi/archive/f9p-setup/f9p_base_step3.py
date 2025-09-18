import serial
import time
import logging
from pyubx2 import UBXMessage, UBXReader

# ---------------- CONFIGURATION ----------------
PORT = "/dev/f9p"
BAUDRATE = 115200
SURVEY_MIN_DUR = 30        # seconds
SURVEY_ACC_LIMIT = 20.0     # meters
SURVEY_TIMEOUT = 900        # max seconds to wait for completion
# ------------------------------------------------

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)

def send_cfg_msg_enable(ser, msg_class, msg_id, rate=1):
    """Enable a UBX message (e.g., NAV-SVIN) on USB port."""
    try:
        payload = {
            "msgClass": msg_class,
            "msgID": msg_id,
            "rateDDC": 0, "rateUART1": 0, "rateUART2": 0,
            "rateUSB": rate, "rateSPI": 0
        }
        msg = UBXMessage("CFG", "CFG-MSG", msgmode=0, **payload)
        ser.write(msg.serialize())
        logging.info(f"Sent CFG-MSG enable for {msg.identity} on USB (rate={rate})")
    except Exception as e:
        logging.error(f"Failed to enable {msg_class}-{msg_id} on USB: {e}")

def send_cfg_tmode3(ser, mode, svin_min_dur=0, svin_acc_limit=0.0):
    """Send a CFG-TMODE3 message to configure Time Mode 3."""
    payload = {
        "version": 0,
        "reserved0": 0,
        "rcvrMode": mode,
        "lla": 0,
        "ecefXOrLat": 0,
        "ecefYOrLon": 0,
        "ecefZOrAlt": 0,
        "ecefXOrLatHP": 0,
        "ecefYOrLonHP": 0,
        "ecefZOrAltHP": 0,
        "reserved1": 0,
        "fixedPosAcc": 0,
        "svinMinDur": svin_min_dur,
        "svinAccLimit": int(svin_acc_limit * 1e4),  # convert m to 0.1 mm
        "reserved2": 0
    }
    msg = UBXMessage("CFG", "CFG-TMODE3", msgmode=0, **payload)
    ser.write(msg.serialize())
    logging.info(f"Sent CFG-TMODE3 (mode={mode}, minDur={svin_min_dur}s, accLimit={svin_acc_limit}m)")

def main():
    logging.info(f"Connecting to F9P on {PORT} @ {BAUDRATE}")
    with serial.Serial(PORT, BAUDRATE, timeout=2) as ser:
        ubr = UBXReader(ser)  # <-- no ERR_IGNORE here

        # Step 1: Disable TMODE3 (ensure clean start)
        send_cfg_tmode3(ser, mode=0)
        time.sleep(1)

        # Step 2: Enable NAV-SVIN output on USB
        send_cfg_msg_enable(ser, msg_class=0x01, msg_id=0x3B, rate=1)  # NAV-SVIN

        # Step 3: Enable Survey-In mode
        send_cfg_tmode3(ser, mode=1, svin_min_dur=SURVEY_MIN_DUR, svin_acc_limit=SURVEY_ACC_LIMIT)
        logging.info("Waiting 2s before starting to monitor NAV-SVIN...")
        time.sleep(2)

        start_time = time.time()
        survey_started = False

        while True:
            if time.time() - start_time > SURVEY_TIMEOUT:
                logging.error("Survey-In timed out without completing.")
                break

            if ser.in_waiting:
                (raw, parsed) = ubr.read()
                if parsed and parsed.identity == "NAV-SVIN":
                    if not survey_started and parsed.active == 1:
                        logging.info("[Survey] Survey-In started.")
                        survey_started = True

                    logging.info(f"[Survey] dur={parsed.dur}s meanAcc={parsed.meanAcc/10000:.4f}m "
                                 f"valid={parsed.valid} active={parsed.active}")

                    if parsed.valid == 1 and parsed.active == 0:
                        logging.info("[Survey] Survey-In completed successfully!")
                        break

            time.sleep(1)

if __name__ == "__main__":
    main()
