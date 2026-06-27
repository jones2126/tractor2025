#!/usr/bin/env python3
"""
F9P Field Survey Tool - Complete & Robust
"""

import serial
import time
import io
from pyubx2 import UBXMessage, UBXReader

PORT = "/dev/f9p"
BAUD = 115200

def main():
    print("🚀 F9P Field Survey Tool (Auto Re-enable)")
    print("=" * 65)

    with serial.Serial(PORT, BAUD, timeout=2) as ser:
        # ====================== 1. START SURVEY ======================
        print("\n[1] Starting Survey-In (min 120s, ≤3.0m, saved to Flash)...")

        # Silence outputs
        silence = []
        for t in ["1005", "1074", "1084", "1094", "1230"]:
            silence += [(f"CFG_MSGOUT_RTCM_3X_TYPE{t}_USB", 0),
                        (f"CFG_MSGOUT_RTCM_3X_TYPE{t}_UART1", 0)]
        ser.write(UBXMessage.config_set(7, 0, silence).serialize())
        time.sleep(1)

        # Configure Survey-In
        cfg = [
            ("CFG_TMODE_MODE", 1),
            ("CFG_NAVSPG_DYNMODEL", 2),           # Stationary
            ("CFG_TMODE_SVIN_MIN_DUR", 120),
            ("CFG_TMODE_SVIN_ACC_LIMIT", 30000),  # 3.0 m
        ]
        ser.write(UBXMessage.config_set(7, 0, cfg).serialize())
        time.sleep(2)
        print("✅ Survey-In started\n")

        # ====================== 2. MONITOR ======================
        print("[2] Monitoring Survey-In every 30 seconds...\n")
        poll = UBXMessage("NAV", "NAV-SVIN", msgmode=0)
        completed = False

        while not completed:
            ser.reset_input_buffer()
            ser.write(poll.serialize())
            ser.flush()
            time.sleep(1.5)

            raw = ser.read(ser.in_waiting or 1024)
            if raw:
                for _, msg in UBXReader(io.BytesIO(raw)):
                    if hasattr(msg, 'identity') and msg.identity == "NAV-SVIN":
                        acc = msg.meanAcc / 10000.0
                        print(f"[{int(time.time())%1000:3d}s] obs={msg.obs:3d}  acc={acc:.2f}m  active={msg.active}")

                        if msg.active == 0 and msg.obs >= 60:
                            print("\n" + "="*70)
                            print("🎉 SURVEY COMPLETE!")
                            print(f"   Observations : {msg.obs}")
                            print(f"   Mean Accuracy: {acc:.2f} m")
                            print(f"   Duration     : {msg.dur} seconds")
                            print("="*70)
                            completed = True
                            break
            time.sleep(28)

        # ====================== 3. RE-ENABLE OUTPUTS ======================
        print("\n[3] Re-enabling RTCM + GGA output...")
        enable = []
        for t in ["1005", "1074", "1084", "1094", "1230"]:
            enable += [(f"CFG_MSGOUT_RTCM_3X_TYPE{t}_USB", 1),
                       (f"CFG_MSGOUT_RTCM_3X_TYPE{t}_UART1", 1)]
        enable += [("CFG_MSGOUT_NMEA_ID_GGA_USB", 1),
                   ("CFG_MSGOUT_NMEA_ID_GGA_UART1", 1)]

        ser.write(UBXMessage.config_set(7, 0, enable).serialize())
        time.sleep(1)
        print("✅ RTCM messages + GGA re-enabled")

    print("\n🎯 BASE STATION IS READY!")
    print("   Run this command:")
    print("   sudo systemctl start rtcm_server")
    print("\n   Check live position:")
    print("   cat current_position.json")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    except Exception as e:
        print(f"\nError: {e}")