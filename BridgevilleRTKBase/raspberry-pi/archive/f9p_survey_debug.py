#!/usr/bin/env python3
"""
Enhanced F9P Survey-In Debugger
===============================
Adds detailed debugging to identify why Survey-In isn't activating.
"""

import sys, time, serial
from pyubx2 import UBXMessage, UBXReader, SET

SURVEY_MIN_DURATION = 120      # seconds
SURVEY_ACC_LIMIT = 50000       # 0.1 mm units (5 m)
SURVEY_MAX_WAIT = 600          # seconds (10 min)
FIX_WAIT_TIMEOUT = 60          # wait up to 60s for 3D fix

# ---------- UBX Helpers ----------

def build_cfg_cfg(clearMask, saveMask, loadMask, deviceMask):
    sync = b"\xB5\x62"
    msg_class = b"\x06"
    msg_id = b"\x09"
    payload = (
        int(clearMask).to_bytes(4, "little")
        + int(saveMask).to_bytes(4, "little")
        + int(loadMask).to_bytes(4, "little")
        + int(deviceMask).to_bytes(1, "little")
    )
    length = len(payload).to_bytes(2, "little")
    ck_a = ck_b = 0
    for byte in msg_class + msg_id + length + payload:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return sync + msg_class + msg_id + length + payload + bytes([ck_a, ck_b])

def send_and_wait_ack(ser, msg, timeout=2.0):  # Increased timeout
    ser.reset_input_buffer()
    ser.write(msg.serialize())
    ser.flush()
    ubr = UBXReader(ser, protfilter=2)
    end = time.time() + timeout
    while time.time() < end:
        if ser.in_waiting:
            try:
                raw, parsed = ubr.read()
                if parsed:
                    print(f"    DEBUG: Received {parsed.identity}")
                    if parsed.identity == "ACK-ACK":
                        print(f"    DEBUG: ACK-ACK for class=0x{parsed.clsID:02X}, id=0x{parsed.msgID:02X}")
                        return True
                    if parsed.identity == "ACK-NAK":
                        print(f"    DEBUG: ACK-NAK for class=0x{parsed.clsID:02X}, id=0x{parsed.msgID:02X}")
                        return False
            except Exception as e:
                print(f"    DEBUG: Parse error: {e}")
        time.sleep(0.05)
    print("    DEBUG: Timeout waiting for ACK")
    return None

def reset_device(ser):
    print("Resetting device to factory defaults...")
    frame = build_cfg_cfg(0xFFFFFFFF, 0x00000000, 0x00000000, 0x17)
    ser.write(frame)
    ser.flush()
    time.sleep(5)  # Increased wait time
    print("  ✓ Reset command sent, waiting for reboot...")

def wait_for_fix(ser):
    print(f"Waiting up to {FIX_WAIT_TIMEOUT}s for 3D GNSS fix...")
    poll = UBXMessage("NAV", "NAV-PVT", msgmode=0)
    ubr = UBXReader(ser, protfilter=2)
    start = time.time()
    
    while time.time() - start < FIX_WAIT_TIMEOUT:
        try:
            ser.reset_input_buffer()
            ser.write(poll.serialize())
            ser.flush()
            time.sleep(0.5)  # Increased wait
            
            if ser.in_waiting:
                raw, parsed = ubr.read()
                if parsed and parsed.identity == "NAV-PVT":
                    fixType = parsed.__dict__.get("fixType", 0)
                    numSV = parsed.__dict__.get("numSV", 0)
                    carrSoln = parsed.__dict__.get("carrSoln", 0)
                    print(f"  ➤ fixType={fixType}, numSV={numSV}, carrSoln={carrSoln}")
                    if fixType >= 3:
                        print("  ✓ 3D fix acquired.")
                        return True
        except Exception as e:
            print(f"  DEBUG: Error polling PVT: {e}")
        time.sleep(2)
        
    print("  ✗ Timed out waiting for 3D fix.")
    return False

def poll_tmode3_detailed(ser):
    """Enhanced TMODE3 polling with detailed output"""
    poll = UBXMessage("CFG", "CFG-TMODE3", msgmode=0)
    ser.reset_input_buffer()
    ser.write(poll.serialize())
    ser.flush()
    time.sleep(0.5)
    
    ubr = UBXReader(ser, protfilter=2)
    timeout = time.time() + 2.0
    
    while time.time() < timeout:
        if ser.in_waiting:
            try:
                raw, parsed = ubr.read()
                if parsed and parsed.identity == "CFG-TMODE3":
                    flags = parsed.__dict__.get("flags", 0)
                    version = parsed.__dict__.get("version", 0)
                    
                    print(f"    DEBUG: TMODE3 Response:")
                    print(f"      version: {version}")
                    print(f"      flags: 0x{flags:04X}")
                    print(f"      mode: {flags & 0x0007} (0=disabled, 1=survey-in, 2=fixed)")
                    print(f"      survey-in active: {bool(flags & 0x0001)}")
                    
                    if flags & 0x0007 == 1:  # Survey-In mode
                        svinMinDur = parsed.__dict__.get("svinMinDur", 0)
                        svinAccLimit = parsed.__dict__.get("svinAccLimit", 0)
                        print(f"      svinMinDur: {svinMinDur}s")
                        print(f"      svinAccLimit: {svinAccLimit/10000:.2f}m")
                    
                    return parsed
            except Exception as e:
                print(f"    DEBUG: Error parsing TMODE3: {e}")
        time.sleep(0.1)
    
    print("    DEBUG: No TMODE3 response received")
    return None

def configure_survey_in_enhanced(ser):
    """Enhanced Survey-In configuration with better error handling"""
    print("Configuring Survey-In with enhanced debugging...")
    
    # First, check current TMODE3 state
    print("  Checking current TMODE3 state...")
    current_state = poll_tmode3_detailed(ser)
    
    # Configure Survey-In
    msg = UBXMessage(
        "CFG", "CFG-TMODE3", msgmode=SET,
        version=0, reserved1=0, flags=0x0001,  # Survey-In mode
        ecefX=0, ecefY=0, ecefZ=0,
        lat=0, lon=0, height=0,
        latHP=0, lonHP=0, heightHP=0,
        reserved2=0, fixedPosAcc=0,
        svinMinDur=SURVEY_MIN_DURATION,
        svinAccLimit=SURVEY_ACC_LIMIT
    )
    
    print("  Sending CFG-TMODE3 (Survey-In)...")
    ack_result = send_and_wait_ack(ser, msg, timeout=3.0)
    
    if ack_result is True:
        print("  ✓ Received ACK for CFG-TMODE3")
    elif ack_result is False:
        print("  ✗ Received NAK for CFG-TMODE3")
        return False
    else:
        print("  ⚠ No response to CFG-TMODE3")
        return False
    
    # Wait a moment for the change to take effect
    print("  Waiting for mode change to take effect...")
    time.sleep(3)
    
    # Verify the configuration
    print("  Verifying Survey-In configuration...")
    new_state = poll_tmode3_detailed(ser)
    
    if new_state:
        flags = new_state.__dict__.get("flags", 0)
        if flags & 0x0001:
            print("  ✓ Survey-In mode successfully activated")
            return True
        else:
            print(f"  ✗ Survey-In not active. Current mode: {flags & 0x0007}")
            return False
    else:
        print("  ✗ Could not verify TMODE3 state")
        return False

def check_nav_status(ser):
    """Check navigation status for additional debugging"""
    print("Checking navigation status...")
    poll = UBXMessage("NAV", "NAV-STATUS", msgmode=0)
    ser.reset_input_buffer()
    ser.write(poll.serialize())
    ser.flush()
    time.sleep(0.5)
    
    ubr = UBXReader(ser, protfilter=2)
    if ser.in_waiting:
        try:
            raw, parsed = ubr.read()
            if parsed and parsed.identity == "NAV-STATUS":
                gpsFix = parsed.__dict__.get("gpsFix", 0)
                flags = parsed.__dict__.get("flags", 0)
                print(f"  NAV-STATUS: gpsFix={gpsFix}, flags=0x{flags:02X}")
                return parsed
        except Exception as e:
            print(f"  Error reading NAV-STATUS: {e}")
    return None

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/f9p"
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print("Enhanced F9P Survey-In Debugger")
    print("=" * 40)
    print(f"Port: {port}\nBaudrate: {baudrate}")

    try:
        with serial.Serial(port, baudrate, timeout=2) as ser:
            time.sleep(1)
            print("✓ Serial connection established")

            # Optional: Skip reset for debugging
            if input("Reset device? (y/n, default=n): ").lower().startswith('y'):
                reset_device(ser)
            
            if not wait_for_fix(ser):
                print("❌ No 3D fix acquired")
                # Continue anyway for debugging
                print("⚠ Continuing without 3D fix for debugging purposes...")
            
            # Check navigation status
            check_nav_status(ser)
            
            # Try enhanced Survey-In configuration
            if configure_survey_in_enhanced(ser):
                print("✓ Survey-In successfully configured!")
                
                # Try to start monitoring
                print("Starting Survey-In monitoring...")
                poll = UBXMessage("NAV", "NAV-SVIN", msgmode=0)
                ser.write(poll.serialize())
                time.sleep(1)
                
                ubr = UBXReader(ser, protfilter=2)
                if ser.in_waiting:
                    try:
                        raw, parsed = ubr.read()
                        if parsed and parsed.identity == "NAV-SVIN":
                            active = parsed.__dict__.get("active", 0)
                            obs = parsed.__dict__.get("obs", 0)
                            print(f"  Survey status: active={active}, obs={obs}")
                    except Exception as e:
                        print(f"  Error reading SVIN: {e}")
            else:
                print("❌ Survey-In configuration failed")

    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()