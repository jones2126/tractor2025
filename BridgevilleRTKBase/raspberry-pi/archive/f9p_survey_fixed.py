#!/usr/bin/env python3
"""
F9P Survey-In Fixed Version
===========================
Addresses the ACK-but-no-activate issue with comprehensive fixes.
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

def send_and_wait_ack(ser, msg, timeout=3.0, debug=False):
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
                    if debug:
                        print(f"    DEBUG: Received {parsed.identity}")
                    if parsed.identity == "ACK-ACK":
                        return True
                    if parsed.identity == "ACK-NAK":
                        return False
            except Exception as e:
                if debug:
                    print(f"    DEBUG: Parse error: {e}")
        time.sleep(0.05)
    return None

def check_firmware_version(ser):
    """Check F9P firmware version for known issues"""
    print("Checking firmware version...")
    poll = UBXMessage("MON", "MON-VER", msgmode=0)
    ser.reset_input_buffer()
    ser.write(poll.serialize())
    ser.flush()
    time.sleep(1)
    
    ubr = UBXReader(ser, protfilter=2)
    timeout = time.time() + 3.0
    
    while time.time() < timeout:
        if ser.in_waiting:
            try:
                raw, parsed = ubr.read()
                if parsed and parsed.identity == "MON-VER":
                    sw_version = parsed.__dict__.get("swVersion", "Unknown")
                    hw_version = parsed.__dict__.get("hwVersion", "Unknown")
                    print(f"  SW Version: {sw_version}")
                    print(f"  HW Version: {hw_version}")
                    
                    # Check for known problematic versions
                    if "1.00" in sw_version or "1.10" in sw_version:
                        print("  ⚠ Warning: This firmware version may have Survey-In issues")
                    
                    return sw_version, hw_version
            except Exception as e:
                print(f"  Error reading version: {e}")
        time.sleep(0.1)
    
    print("  Could not read firmware version")
    return None, None

def comprehensive_reset(ser):
    """More thorough reset procedure"""
    print("Performing comprehensive reset...")
    
    # 1. Factory reset
    print("  Step 1: Factory reset...")
    frame = build_cfg_cfg(0xFFFFFFFF, 0x00000000, 0x00000000, 0x17)
    ser.write(frame)
    ser.flush()
    time.sleep(3)
    
    # 2. Disable any existing TMODE3 explicitly
    print("  Step 2: Explicitly disabling TMODE3...")
    disable_msg = UBXMessage(
        "CFG", "CFG-TMODE3", msgmode=SET,
        version=0, reserved1=0, flags=0x0000,  # Disabled
        ecefX=0, ecefY=0, ecefZ=0, lat=0, lon=0, height=0,
        latHP=0, lonHP=0, heightHP=0, reserved2=0, fixedPosAcc=0,
        svinMinDur=0, svinAccLimit=0
    )
    send_and_wait_ack(ser, disable_msg, timeout=3.0)
    time.sleep(2)
    
    # 3. Reset navigation settings
    print("  Step 3: Resetting navigation settings...")
    nav_reset = UBXMessage("CFG", "CFG-NAV5", msgmode=SET,
                          mask=0x0001, dynModel=0)  # Portable dynamic model
    send_and_wait_ack(ser, nav_reset, timeout=3.0)
    time.sleep(2)
    
    print("  ✓ Comprehensive reset completed")

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
            time.sleep(0.5)
            
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
    """Poll TMODE3 with detailed debugging"""
    poll = UBXMessage("CFG", "CFG-TMODE3", msgmode=0)
    ser.reset_input_buffer()
    ser.write(poll.serialize())
    ser.flush()
    time.sleep(0.5)
    
    ubr = UBXReader(ser, protfilter=2)
    timeout = time.time() + 3.0
    
    while time.time() < timeout:
        if ser.in_waiting:
            try:
                raw, parsed = ubr.read()
                if parsed and parsed.identity == "CFG-TMODE3":
                    flags = parsed.__dict__.get("flags", 0)
                    print(f"    TMODE3 flags: 0x{flags:04X}, mode: {flags & 0x0007}")
                    return parsed
            except Exception as e:
                print(f"    Error parsing TMODE3: {e}")
        time.sleep(0.1)
    
    print("    No TMODE3 response received")
    return None

def alternative_survey_config_method(ser):
    """Alternative method using CFG-VALSET (newer configuration interface)"""
    print("Trying alternative configuration method (CFG-VALSET)...")
    
    try:
        # Use CFG-VALSET to configure Survey-In
        # This is the newer configuration interface that might work better
        valset_msg = UBXMessage("CFG", "CFG-VALSET", msgmode=SET,
                               version=1, layers=0x01, reserved1=0,
                               cfgData=[
                                   # TMODE_MODE = 1 (Survey-In)
                                   (0x20030001, 1),
                                   # TMODE_SVIN_MIN_DUR (seconds)
                                   (0x40030010, SURVEY_MIN_DURATION),
                                   # TMODE_SVIN_ACC_LIMIT (0.1mm units)
                                   (0x40030011, SURVEY_ACC_LIMIT)
                               ])
        
        ack = send_and_wait_ack(ser, valset_msg, timeout=3.0, debug=True)
        if ack:
            print("  ✓ CFG-VALSET acknowledged")
            time.sleep(3)
            return True
        else:
            print("  ✗ CFG-VALSET not acknowledged")
            return False
            
    except Exception as e:
        print(f"  CFG-VALSET method failed: {e}")
        return False

def multi_attempt_survey_config(ser):
    """Multiple approaches to configure Survey-In"""
    print("Attempting Survey-In configuration with multiple methods...")
    
    # Method 1: Standard CFG-TMODE3
    print("\n--- Method 1: Standard CFG-TMODE3 ---")
    for attempt in range(3):
        print(f"Attempt {attempt + 1}...")
        
        # Configure Survey-In
        msg = UBXMessage(
            "CFG", "CFG-TMODE3", msgmode=SET,
            version=0, reserved1=0, flags=0x0001,  # Survey-In
            ecefX=0, ecefY=0, ecefZ=0, lat=0, lon=0, height=0,
            latHP=0, lonHP=0, heightHP=0, reserved2=0, fixedPosAcc=0,
            svinMinDur=SURVEY_MIN_DURATION, svinAccLimit=SURVEY_ACC_LIMIT
        )
        
        ack = send_and_wait_ack(ser, msg, timeout=3.0)
        if ack:
            print("  ✓ CFG-TMODE3 acknowledged")
        else:
            print("  ✗ CFG-TMODE3 not acknowledged")
            continue
            
        # Wait and verify
        time.sleep(5)  # Longer wait
        result = poll_tmode3_detailed(ser)
        
        if result:
            flags = result.__dict__.get("flags", 0)
            if flags & 0x0001:
                print("  ✓ Survey-In mode activated!")
                return True
        
        print(f"  ⚠ Attempt {attempt + 1} failed, Survey-In not active")
        time.sleep(2)
    
    # Method 2: Alternative CFG-VALSET
    print("\n--- Method 2: CFG-VALSET ---")
    if alternative_survey_config_method(ser):
        time.sleep(3)
        result = poll_tmode3_detailed(ser)
        if result:
            flags = result.__dict__.get("flags", 0)
            if flags & 0x0001:
                print("  ✓ Survey-In activated via CFG-VALSET!")
                return True
    
    # Method 3: Raw UBX command
    print("\n--- Method 3: Raw UBX approach ---")
    try:
        # Manually craft UBX message
        raw_cmd = bytes([
            0xB5, 0x62,  # sync
            0x06, 0x71,  # CFG-TMODE3
            0x28, 0x00,  # length = 40
            # payload
            0x00, 0x00,  # version, reserved1
            0x01, 0x00,  # flags (Survey-In mode)
            0x00, 0x00, 0x00, 0x00,  # ecefX
            0x00, 0x00, 0x00, 0x00,  # ecefY
            0x00, 0x00, 0x00, 0x00,  # ecefZ
            0x00, 0x00, 0x00, 0x00,  # lat
            0x00, 0x00, 0x00, 0x00,  # lon
            0x00, 0x00, 0x00, 0x00,  # height
            0x00, 0x00, 0x00, 0x00,  # latHP, lonHP, heightHP, reserved2
            0x00, 0x00, 0x00, 0x00,  # fixedPosAcc
            SURVEY_MIN_DURATION & 0xFF, (SURVEY_MIN_DURATION >> 8) & 0xFF,
            (SURVEY_MIN_DURATION >> 16) & 0xFF, (SURVEY_MIN_DURATION >> 24) & 0xFF,  # svinMinDur
            SURVEY_ACC_LIMIT & 0xFF, (SURVEY_ACC_LIMIT >> 8) & 0xFF,
            (SURVEY_ACC_LIMIT >> 16) & 0xFF, (SURVEY_ACC_LIMIT >> 24) & 0xFF  # svinAccLimit
        ])
        
        # Calculate checksum
        ck_a = ck_b = 0
        for byte in raw_cmd[2:-2]:  # Skip sync and checksum
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        
        raw_cmd += bytes([ck_a, ck_b])
        
        ser.write(raw_cmd)
        ser.flush()
        time.sleep(5)
        
        result = poll_tmode3_detailed(ser)
        if result:
            flags = result.__dict__.get("flags", 0)
            if flags & 0x0001:
                print("  ✓ Survey-In activated via raw command!")
                return True
                
    except Exception as e:
        print(f"  Raw command method failed: {e}")
    
    print("❌ All Survey-In configuration methods failed")
    return False

def poll_survey_status(ser):
    poll = UBXMessage("NAV", "NAV-SVIN", msgmode=0)
    ser.write(poll.serialize())
    ser.flush()
    time.sleep(0.2)
    ubr = UBXReader(ser, protfilter=2)
    if ser.in_waiting:
        try:
            raw, parsed = ubr.read()
            if parsed and parsed.identity == "NAV-SVIN":
                return parsed
        except:
            pass
    return None

def wait_for_survey_completion(ser):
    print(f"Monitoring Survey-In progress (timeout {SURVEY_MAX_WAIT}s)...")
    start = time.time()
    last_obs = 0
    
    while time.time() - start < SURVEY_MAX_WAIT:
        status = poll_survey_status(ser)
        if status:
            active = status.__dict__.get("active", 0)
            obs = status.__dict__.get("obs", 0)
            meanAcc = status.__dict__.get("meanAcc", 0) / 10000.0
            
            # Only print if observations changed to reduce spam
            if obs != last_obs or obs % 30 == 0:
                elapsed = int(time.time() - start)
                print(f"  [{elapsed:3d}s] active={active}, obs={obs}, meanAcc={meanAcc:.3f}m")
                last_obs = obs
            
            if not active and obs > 0:
                print("  ✓ Survey-In completed successfully!")
                return True
        time.sleep(1)
        
    print("  ✗ Survey-In timed out after 10 minutes")
    return False

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/f9p"
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print("F9P Survey-In Comprehensive Fix")
    print("=" * 40)
    print(f"Port: {port}\nBaudrate: {baudrate}")

    try:
        with serial.Serial(port, baudrate, timeout=2) as ser:
            time.sleep(1)
            print("✓ Serial connection established")

            # Check firmware version
            check_firmware_version(ser)
            
            # Comprehensive reset
            comprehensive_reset(ser)
            
            # Wait for fix
            if not wait_for_fix(ser):
                print("❌ No 3D fix acquired - Survey-In requires 3D fix")
                return
            
            # Multi-method Survey-In configuration
            if multi_attempt_survey_config(ser):
                print("\n✓ Survey-In successfully configured!")
                
                # Monitor survey progress
                if wait_for_survey_completion(ser):
                    print("🎉 Survey-In completed! Device is now a base station.")
                else:
                    print("⚠ Survey-In started but didn't complete in time")
            else:
                print("\n❌ Could not activate Survey-In mode")
                print("This may be a firmware issue or hardware problem.")

    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()