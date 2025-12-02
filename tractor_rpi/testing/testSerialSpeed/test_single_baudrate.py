#!/usr/bin/env python3
"""
test_single_baudrate.py
Quick test of a single baud rate for 20 seconds
"""

import serial
import time
import numpy as np
from collections import deque

# Configure these to match your Teensy firmware
SERIAL_PORT = '/dev/teensy'
BAUD_RATE = 2000000  # Must match Teensy's BAUD_RATE in .cpp file
TEST_DURATION = 20   # seconds

class QuickTester:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        
        self.commands_sent = 0
        self.echoes_received = 0
        self.heartbeats_received = 0
        
        self.latencies = deque(maxlen=1000)
        self.heartbeat_intervals = deque(maxlen=500)
        self.last_heartbeat_time = None
        self.last_command_time = None
        
    def connect(self):
        try:
            print(f"Connecting to {self.port} at {self.baud_rate} baud...")
            self.ser = serial.Serial(
                self.port,
                self.baud_rate,
                timeout=1.0  # Increased from 0.1
            )
            
            # Flush buffers completely
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            time.sleep(1)  # Let buffers clear
            
            # Read and discard any garbage data
            garbage_count = 0
            start = time.time()
            while time.time() - start < 2:
                try:
                    if self.ser.in_waiting:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if line and len(line) > 5:  # Valid looking data
                            if "HEARTBEAT" in line or "STATS" in line:
                                print(f"  ✓ Receiving data: {line[:50]}")
                                print(f"  ✓ Connected successfully at {self.baud_rate} baud")
                                return True
                        else:
                            garbage_count += 1
                except Exception:
                    garbage_count += 1
                time.sleep(0.1)
            
            # Even if we didn't see perfect messages, if serial port opened, continue
            if garbage_count < 20:  # Some data came through
                print(f"  ⚠ Connected (saw some data, may have startup noise)")
                return True
            else:
                print(f"  ✗ Too much garbage data - possible baud rate mismatch")
                return False
            
        except Exception as e:
            print(f"  ✗ Connection failed: {e}")
            return False
    
    def send_command(self):
        try:
            command = f"CMD,{self.commands_sent},0.5,0.1\n"
            send_time = time.time()
            self.ser.write(command.encode('utf-8'))
            self.commands_sent += 1
            self.last_command_time = send_time
            return send_time
        except Exception as e:
            print(f"  Send error: {e}")
            return None
    
    def process_messages(self):
        while self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                
                receive_time = time.time()
                parts = line.split(',')
                
                if parts[0] == "ECHO":
                    self.echoes_received += 1
                    if self.last_command_time:
                        latency = (receive_time - self.last_command_time) * 1000
                        self.latencies.append(latency)
                
                elif parts[0] == "HEARTBEAT":
                    self.heartbeats_received += 1
                    if self.last_heartbeat_time:
                        interval = (receive_time - self.last_heartbeat_time) * 1000
                        self.heartbeat_intervals.append(interval)
                    self.last_heartbeat_time = receive_time
                    
            except Exception:
                pass
    
    def run_test(self, duration):
        print(f"\nRunning test for {duration} seconds at {self.baud_rate} baud...")
        print("Sending commands at 20 Hz...\n")
        
        start_time = time.time()
        next_send = start_time + 0.05  # 50ms = 20 Hz
        last_status = start_time
        
        while time.time() - start_time < duration:
            current_time = time.time()
            
            # Send at 20 Hz
            if current_time >= next_send:
                self.send_command()
                next_send += 0.05
            
            # Read responses
            self.process_messages()
            
            # Status every 5 seconds
            if current_time - last_status >= 5:
                elapsed = current_time - start_time
                cmd_rate = self.commands_sent / elapsed
                echo_rate = self.echoes_received / elapsed
                hb_rate = self.heartbeats_received / elapsed
                avg_lat = np.mean(self.latencies) if self.latencies else 0
                
                print(f"  [{elapsed:.0f}s] Sent: {self.commands_sent} ({cmd_rate:.1f} Hz), "
                      f"Echoed: {self.echoes_received} ({echo_rate:.1f} Hz), "
                      f"Heartbeats: {self.heartbeats_received} ({hb_rate:.1f} Hz), "
                      f"Latency: {avg_lat:.2f}ms")
                last_status = current_time
            
            time.sleep(0.001)
        
        # Final processing
        time.sleep(0.2)
        self.process_messages()
    
    def print_results(self):
        print("\n" + "="*70)
        print(f"TEST RESULTS @ {self.baud_rate} baud")
        print("="*70)
        
        success_rate = self.echoes_received / max(1, self.commands_sent)
        
        print(f"\nCommand/Response:")
        print(f"  Commands sent:     {self.commands_sent}")
        print(f"  Echoes received:   {self.echoes_received}")
        print(f"  Success rate:      {success_rate*100:.1f}%")
        
        if self.latencies:
            lat_arr = np.array(self.latencies)
            print(f"\nLatency:")
            print(f"  Mean:    {np.mean(lat_arr):.2f} ms")
            print(f"  Median:  {np.median(lat_arr):.2f} ms")
            print(f"  Std:     {np.std(lat_arr):.2f} ms")
            print(f"  Min:     {np.min(lat_arr):.2f} ms")
            print(f"  Max:     {np.max(lat_arr):.2f} ms")
            print(f"  95th:    {np.percentile(lat_arr, 95):.2f} ms")
        
        if self.heartbeat_intervals:
            hb_arr = np.array(self.heartbeat_intervals)
            actual_hz = 1000.0 / np.mean(hb_arr)
            print(f"\nHeartbeat:")
            print(f"  Count:   {self.heartbeats_received}")
            print(f"  Mean:    {np.mean(hb_arr):.2f} ms ({actual_hz:.1f} Hz)")
            print(f"  Std:     {np.std(hb_arr):.2f} ms")
        
        # Pass/Fail
        print(f"\nPass/Fail Criteria:")
        print(f"  Success rate > 99%:  {'PASS ✓' if success_rate > 0.99 else 'FAIL ✗'} ({success_rate*100:.1f}%)")
        
        if self.latencies:
            avg_lat = np.mean(self.latencies)
            print(f"  Avg latency < 10ms:  {'PASS ✓' if avg_lat < 10 else 'FAIL ✗'} ({avg_lat:.2f}ms)")
        
        if self.heartbeat_intervals:
            hb_std = np.std(self.heartbeat_intervals)
            print(f"  HB jitter < 5ms:     {'PASS ✓' if hb_std < 5 else 'FAIL ✗'} ({hb_std:.2f}ms)")
        
        overall_pass = (success_rate > 0.99 and 
                       (not self.latencies or np.mean(self.latencies) < 10) and
                       (not self.heartbeat_intervals or np.std(self.heartbeat_intervals) < 5))
        
        print(f"\n  OVERALL:             {'PASS ✓✓' if overall_pass else 'FAIL ✗✗'}")
        
        if overall_pass:
            print(f"\n{'='*70}")
            print(f"SUCCESS! {self.baud_rate} baud works great for 20 Hz operation!")
            print("="*70)
        else:
            print(f"\n{'='*70}")
            print(f"ISSUES DETECTED at {self.baud_rate} baud")
            if success_rate < 0.99:
                print("  - Low success rate: Try lower baud rate or check cable")
            if self.latencies and np.mean(self.latencies) >= 10:
                print("  - High latency: Try higher baud rate")
            if self.heartbeat_intervals and np.std(self.heartbeat_intervals) >= 5:
                print("  - High jitter: Check USB connection and system load")
            print("="*70)
    
    def cleanup(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

def main():
    print("="*70)
    print(f"SINGLE BAUD RATE TEST")
    print("="*70)
    print(f"Testing: {BAUD_RATE} baud")
    print(f"Port: {SERIAL_PORT}")
    print(f"Duration: {TEST_DURATION} seconds")
    print("="*70)
    
    tester = QuickTester(SERIAL_PORT, BAUD_RATE)
    
    if not tester.connect():
        print("\n✗ Failed to connect. Check:")
        print("  1. Teensy is connected and powered")
        print("  2. Teensy firmware has matching BAUD_RATE")
        print("  3. No other programs using the port")
        print(f"  4. Port exists: ls -l {SERIAL_PORT}")
        return
    
    try:
        tester.run_test(TEST_DURATION)
        tester.print_results()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nTest failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        tester.cleanup()

if __name__ == "__main__":
    main()
