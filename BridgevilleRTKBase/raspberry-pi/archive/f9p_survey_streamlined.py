#!/usr/bin/env python3
"""
Streamlined F9P Survey-In Fix
============================
Focused fix for firmware 1.00 Survey-In issues without hanging.
"""

import sys, time, serial
from pyubx2 import UBXMessage, UBXReader, SET

SURVEY_MIN_DURATION = 120      # seconds
SURVEY_ACC_LIMIT = 50000       # 0.1 mm units (5 m)
SURVEY_MAX_WAIT = 600          # seconds (10 min)
FIX_WAIT_TIMEOUT = 30          # reduced timeout

# ---------- UBX Helpers ----------

def send_and_wait_ack(ser, msg, timeout=2.0):
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
                    if parsed.identity == "ACK-ACK":
                        return True
                    if parsed.identity == "ACK-NAK":
                        return False
            except:
                pass
        time.sleep(0.05)
    return None

def wait_for_fix_quick(ser):
    print(f"Checking for 3D GNSS fix...")
    poll = UBXMessage("NAV", "NAV-PVT", msgmode=0)
    ubr = UBXReader(ser, protfilter=2)
    
    for attempt in range(5):  # Only 5 attempts
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
                    print(f"  fixType={fixType}, numSV={numSV}")
                    if fixType >= 3:
                        print("  ✓ 3D fix confirmed")
                        return True
        except:
            pass
        time.sleep(1)
        
    print("  ⚠ Proceeding without confirmed 3D fix")
    return False

def poll_tmode3_simple(ser):
    """Simple TMODE3 poll without hanging"""
    poll = UBXMessage("CFG", "CFG-TMODE3", msgmode=0)
    ser.reset_input_buffer()
    ser.write(poll.serialize())
    ser.flush()
    time.sleep(0.3)
    
    ubr = UBXReader(ser, protfilter=2)
    for _ in range(10):  # Max 10 attempts
        if ser.in_waiting:
            try:
                raw, parsed = ubr.read()
                if parsed and parsed.identity == "CFG-TMODE3":
                    flags = parsed.__dict__.get("flags", 0)
                    return flags
            except:
                pass
        time.sleep(0.1)
    return None

def firmware_1_00_survey_workaround(ser):
    """Specific workaround for firmware 1.00 Survey-In bug"""
    print("Applying firmware 1.00 Survey-In workaround...")
    
    # Step 1: Explicitly disable TMODE3 first
    print("  Step 1: Disabling any existing TMODE3...")
    disable_msg = UBXMessage(
        "CFG", "CFG-TMODE3", msgmode=SET,
        version=0, reserved1=0, flags=0x0000,  # Disabled
        ecefX=0, ecefY=0, ecefZ=0, lat=0, lon=0, height=0,
        latHP=0, lonHP=0, heightHP=0, reserved2=0, fixedPosAcc=0,
        svinMinDur=0, svinAccLimit=0
    )
    send_and_wait_ack(ser, disable_msg)
    time.sleep(1)
    
    # Step 2: Reset dynamic model to ensure clean state
    print("  Step 2: Setting dynamic model...")
    try:
        nav_msg = UBXMessage("CFG", "CFG-NAV5", msgmode=SET,
                            mask=0x0001, dynModel=2)  # Stationary model for base
        send_and_wait_ack(ser, nav_msg)
        time.sleep(1)
    except:
        pass  # Not critical if this fails
    
    # Step 3: Enable Survey-In with specific timing for fw 1.00
    print("  Step 3: Enabling Survey-In...")
    for attempt in range(5):
        survey_msg = UBXMessage(
            "CFG", "CFG-TMODE3", msgmode=SET,
            version=0, reserved1=0, flags=0x0001,  # Survey-In
            ecefX=0, ecefY=0, ecefZ=0, lat=0, lon=0, height=0,
            latHP=0, lonHP=0, heightHP=0, reserved2=0, fixedPosAcc=0,
            svinMinDur=SURVEY_MIN_DURATION, svinAccLimit=SURVEY_ACC_LIMIT
        )
        
        ack = send_and_wait_ack(ser, survey_msg)
        print(f"    Attempt {attempt+1}: {'ACK' if ack else 'NAK/Timeout'}")
        
        if ack:
            # Critical: Wait longer for fw 1.00 to process
            time.sleep(3)
            
            # Check if it actually activated
            flags = poll_tmode3_simple(ser)
            if flags is not None:
                mode = flags & 0x0007
                print(f"    TMODE3 mode: {mode} (0=disabled, 1=survey-in)")
                if mode == 1:
                    print("  ✓ Survey-In successfully activated!")
                    return True
            
        time.sleep(2)  # Wait between attempts
    
    # Step 4: Try alternative approach using raw bytes
    print("  Step 4: Trying raw UBX approach...")
    try:
        # Manually crafted UBX CFG-TMODE3 for Survey-In
        raw_survey = bytearray([
            0xB5, 0x62,  # UBX sync
            0x06, 0x71,  # CFG-TMODE3
            0x28, 0x00,  # length 40
            # Payload:
            0x00, 0x00,  # version=0, reserved1=0
            0x01, 0x00,  # flags=0x0001 (Survey-In mode)
            # All position fields = 0 (24 bytes)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            # fixedPosAcc = 0 (4 bytes)
            0x00, 0x00, 0x00, 0x00,
            # svinMinDur = 120 seconds (4 bytes, little endian)
            0x78, 0x00, 0x00, 0x00,
            # svinAccLimit = 50000 (4 bytes, little endian)  
            0x50, 0xC3, 0x00, 0x00
        ])
        
        # Calculate checksum
        ck_a = ck_b = 0
        for i in range(2, len(raw_survey)):
            ck_a = (ck_a + raw_survey[i]) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        
        raw_survey.extend([ck_a, ck_b])
        
        ser.write(raw_survey)
        ser.flush()
        time.sleep(3)
        
        flags = poll_tmode3_simple(ser)
        if flags is not None and (flags & 0x0007) == 1:
            print("  ✓ Raw UBX method succeeded!")
            return True
            
    except Exception as e:
        print(f"  Raw method failed: {e}")
    
    print("  ✗ All workaround methods failed")
    return False

def monitor_survey_simple(ser):
    """Simple survey monitoring without hanging"""
    print("Monitoring Survey-In (will check every 10 seconds)...")
    poll = UBXMessage("NAV", "NAV-SVIN", msgmode=0)
    start_time = time.time()
    
    for check in range(60):  # Check up to 60 times (10 minutes)
        try:
            ser.reset_input_buffer()
            ser.write(poll.serialize())
            ser.flush()
            time.sleep(0.3)
            
            if ser.in_waiting:
                ubr = UBXReader(ser, protfilter=2)
                raw, parsed = ubr.read()
                if parsed and parsed.identity == "NAV-SVIN":
                    active = parsed.__dict__.get("active", 0)
                    obs = parsed.__dict__.get("obs", 0)
                    meanAcc = parsed.__dict__.get("meanAcc", 0) / 10000.0
                    elapsed = int(time.time() - start_time)
                    
                    print(f"  [{elapsed:3d}s] obs={obs}, meanAcc={meanAcc:.2f}m, active={active}")
                    
                    if not active and obs > 0:
                        print("  ✓ Survey-In completed!")
                        return True
                        
        except Exception as e:
            print(f"  Monitor error: {e}")
            
        time.sleep(10)  # Wait 10 seconds between checks
    
    print("  ⚠ Survey monitoring timed out")
    return False

def enable_rtcm_simple(ser):
    """Simple RTCM enabling without extensive error checking"""
    print("Enabling basic RTCM messages...")
    
    rtcm_msgs = [
        (0x05, "1005 - Base ARP"),
        (0x4A, "1074 - GPS MSM4"),
        (0x54, "1084 - GLONASS MSM4")
    ]
    
    for msgID, name in rtcm_msgs:
        try:
            msg = UBXMessage("CFG", "CFG-MSG", msgmode=SET,
                           msgClass=0xF5, msgID=msgID,
                           rateDDC=0, rateUART1=1, rateUART2=0, rateUSB=1,
                           rateSPI=0, rateI2C=0)
            ack = send_and_wait_ack(ser, msg, timeout=1.0)
            print(f"  {name}: {'✓' if ack else '✗'}")
        except:
            print(f"  {name}: ✗")

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/f9p"
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print("Streamlined F9P Survey-In Fix (Firmware 1.00)")
    print("=" * 50)
    print(f"Port: {port}, Baudrate: {baudrate}")

    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            time.sleep(0.5)
            print("✓ Connected")

            # Quick 3D fix check
            wait_for_fix_quick(ser)
            
            # Apply firmware 1.00 specific workaround
            if firmware_1_00_survey_workaround(ser):
                print("\n🎯 Survey-In activated! Starting monitoring...")
                
                # Simple survey monitoring
                if monitor_survey_simple(ser):
                    print("\n🎉 Survey completed! Enabling RTCM...")
                    enable_rtcm_simple(ser)
                    print("✅ Base station configured!")
                else:
                    print("\n⚠ Survey didn't complete but may still be running")
                    
            else:
                print("\n❌ Could not activate Survey-In")
                print("This firmware version (1.00) has known Survey-In issues.")
                print("Consider updating firmware or using u-center for manual config.")

    except KeyboardInterrupt:
        print("\n⚠ Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()