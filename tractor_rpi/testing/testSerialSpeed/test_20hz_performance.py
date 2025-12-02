#!/usr/bin/env python3
"""
test_20hz_performance.py
Specialized test for consistent 20 Hz bidirectional communication
"""

import serial
import time
import numpy as np
from collections import deque
import sys

SERIAL_PORT = '/dev/teensy'
BAUD_RATE = 2000000  # Use best rate from latency test
TARGET_HZ = 20
TEST_DURATION = 30  # seconds

class Hz20Tester:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        
        # Timing tracking
        self.command_send_times = deque(maxlen=1000)
        self.echo_receive_times = deque(maxlen=1000)
        self.heartbeat_times = deque(maxlen=1000)
        
        # Round-trip times
        self.round_trip_times = deque(maxlen=1000)
        self.pending_commands = {}  # seq: send_time
        
        # Counters
        self.commands_sent = 0
        self.echoes_received = 0
        self.heartbeats_received = 0
        self.command_seq = 0
        
        # Interval analysis
        self.send_intervals = deque(maxlen=500)
        self.echo_intervals = deque(maxlen=500)
        self.heartbeat_intervals = deque(maxlen=500)
        
        self.last_send_time = None
        self.last_echo_time = None
        self.last_heartbeat_time = None
        
    def connect(self):
        """Open serial connection"""
        try:
            self.ser = serial.Serial(
                self.port,
                self.baud_rate,
                timeout=0.01,
                write_timeout=0.01
            )
            time.sleep(2)
            
            # Flush buffers
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            print(f"Connected at {self.baud_rate} baud")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def send_command(self, linear_x, angular_z):
        """Send command with sequence number"""
        try:
            self.command_seq += 1
            command = f"CMD,{self.command_seq},{linear_x:.4f},{angular_z:.4f}\n"
            
            send_time = time.time()
            self.ser.write(command.encode('utf-8'))
            
            self.commands_sent += 1
            self.command_send_times.append(send_time)
            self.pending_commands[self.command_seq] = send_time
            
            # Track send intervals
            if self.last_send_time:
                interval = (send_time - self.last_send_time) * 1000
                self.send_intervals.append(interval)
            self.last_send_time = send_time
            
            return True
        except Exception as e:
            print(f"Send error: {e}")
            return False
    
    def process_messages(self):
        """Process all available messages"""
        while self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                
                receive_time = time.time()
                parts = line.split(',')
                
                if parts[0] == "ECHO" and len(parts) >= 4:
                    # ECHO,timestamp,count,seq,linear_x,angular_z
                    try:
                        seq = int(parts[3]) if len(parts) > 3 else None
                        
                        self.echoes_received += 1
                        self.echo_receive_times.append(receive_time)
                        
                        # Calculate round-trip time
                        if seq and seq in self.pending_commands:
                            send_time = self.pending_commands.pop(seq)
                            rtt = (receive_time - send_time) * 1000  # ms
                            self.round_trip_times.append(rtt)
                        
                        # Track echo intervals
                        if self.last_echo_time:
                            interval = (receive_time - self.last_echo_time) * 1000
                            self.echo_intervals.append(interval)
                        self.last_echo_time = receive_time
                    except:
                        pass
                
                elif parts[0] == "HEARTBEAT":
                    self.heartbeats_received += 1
                    self.heartbeat_times.append(receive_time)
                    
                    # Track heartbeat intervals
                    if self.last_heartbeat_time:
                        interval = (receive_time - self.last_heartbeat_time) * 1000
                        self.heartbeat_intervals.append(interval)
                    self.last_heartbeat_time = receive_time
                    
            except Exception as e:
                pass
    
    def run_test(self, duration):
        """Run test at exactly 20 Hz"""
        print(f"\nRunning {TARGET_HZ} Hz test for {duration} seconds...")
        print("Monitoring timing consistency...\n")
        
        interval = 1.0 / TARGET_HZ
        start_time = time.time()
        next_send_time = start_time + interval
        
        last_status = start_time
        
        while time.time() - start_time < duration:
            current_time = time.time()
            
            # Send command at exact intervals
            if current_time >= next_send_time:
                linear_x = 0.5
                angular_z = 0.1
                self.send_command(linear_x, angular_z)
                next_send_time += interval
                
                # Detect if we're falling behind
                if time.time() > next_send_time:
                    print(f"  Warning: Falling behind schedule at {current_time-start_time:.1f}s")
            
            # Process incoming messages
            self.process_messages()
            
            # Status update every 5 seconds
            if current_time - last_status >= 5:
                self.print_interim_status(current_time - start_time)
                last_status = current_time
            
            # Small sleep to prevent CPU spinning
            sleep_time = max(0, next_send_time - time.time() - 0.001)
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        # Final processing
        time.sleep(0.2)
        self.process_messages()
    
    def print_interim_status(self, elapsed):
        """Print status during test"""
        cmd_rate = self.commands_sent / elapsed
        echo_rate = self.echoes_received / elapsed
        hb_rate = self.heartbeats_received / elapsed
        
        avg_rtt = np.mean(self.round_trip_times) if self.round_trip_times else 0
        
        print(f"  [{elapsed:.0f}s] Sent: {self.commands_sent} ({cmd_rate:.1f} Hz), "
              f"Echoed: {self.echoes_received} ({echo_rate:.1f} Hz), "
              f"Heartbeats: {self.heartbeats_received} ({hb_rate:.1f} Hz), "
              f"RTT: {avg_rtt:.2f}ms")
    
    def analyze_results(self):
        """Detailed analysis of timing consistency"""
        print("\n" + "="*70)
        print("DETAILED TIMING ANALYSIS")
        print("="*70)
        
        # Command sending analysis
        if self.send_intervals:
            send_arr = np.array(self.send_intervals)
            print(f"\nCommand Send Timing (target: {1000/TARGET_HZ:.1f}ms):")
            print(f"  Mean:   {np.mean(send_arr):.2f} ms")
            print(f"  Median: {np.median(send_arr):.2f} ms")
            print(f"  Std:    {np.std(send_arr):.2f} ms")
            print(f"  Min:    {np.min(send_arr):.2f} ms")
            print(f"  Max:    {np.max(send_arr):.2f} ms")
            print(f"  95th:   {np.percentile(send_arr, 95):.2f} ms")
            
            # Jitter analysis
            jitter = np.abs(send_arr - 50.0)
            print(f"  Jitter: {np.mean(jitter):.2f} ms (avg deviation from 50ms)")
            
        # Echo receive timing analysis
        if self.echo_intervals:
            echo_arr = np.array(self.echo_intervals)
            print(f"\nEcho Receive Timing:")
            print(f"  Mean:   {np.mean(echo_arr):.2f} ms")
            print(f"  Median: {np.median(echo_arr):.2f} ms")
            print(f"  Std:    {np.std(echo_arr):.2f} ms")
            
        # Round-trip time analysis
        if self.round_trip_times:
            rtt_arr = np.array(self.round_trip_times)
            print(f"\nRound-Trip Time:")
            print(f"  Mean:   {np.mean(rtt_arr):.2f} ms")
            print(f"  Median: {np.median(rtt_arr):.2f} ms")
            print(f"  Std:    {np.std(rtt_arr):.2f} ms")
            print(f"  Min:    {np.min(rtt_arr):.2f} ms")
            print(f"  Max:    {np.max(rtt_arr):.2f} ms")
            print(f"  95th:   {np.percentile(rtt_arr, 95):.2f} ms")
            
        # Heartbeat analysis
        if self.heartbeat_intervals:
            hb_arr = np.array(self.heartbeat_intervals)
            actual_hz = 1000.0 / np.mean(hb_arr) if np.mean(hb_arr) > 0 else 0
            print(f"\nHeartbeat Timing (target: 50ms @ 20 Hz):")
            print(f"  Mean:   {np.mean(hb_arr):.2f} ms ({actual_hz:.1f} Hz)")
            print(f"  Median: {np.median(hb_arr):.2f} ms")
            print(f"  Std:    {np.std(hb_arr):.2f} ms")
            print(f"  Min:    {np.min(hb_arr):.2f} ms")
            print(f"  Max:    {np.max(hb_arr):.2f} ms")
            
        # Overall statistics
        print(f"\nOverall Statistics:")
        print(f"  Commands sent:      {self.commands_sent}")
        print(f"  Echoes received:    {self.echoes_received}")
        print(f"  Success rate:       {100*self.echoes_received/max(1,self.commands_sent):.1f}%")
        print(f"  Pending commands:   {len(self.pending_commands)}")
        print(f"  Heartbeats:         {self.heartbeats_received}")
        
        # Pass/Fail criteria
        print(f"\nPass/Fail Criteria:")
        success_rate = self.echoes_received / max(1, self.commands_sent)
        avg_jitter = np.mean(np.abs(np.array(self.send_intervals) - 50.0)) if self.send_intervals else 999
        avg_rtt = np.mean(self.round_trip_times) if self.round_trip_times else 999
        
        print(f"  Success rate > 99%:     {'PASS' if success_rate > 0.99 else 'FAIL'} ({success_rate*100:.1f}%)")
        print(f"  Avg jitter < 2ms:       {'PASS' if avg_jitter < 2 else 'FAIL'} ({avg_jitter:.2f}ms)")
        print(f"  Avg RTT < 10ms:         {'PASS' if avg_rtt < 10 else 'FAIL'} ({avg_rtt:.2f}ms)")
        
        overall_pass = success_rate > 0.99 and avg_jitter < 2 and avg_rtt < 10
        print(f"\n  OVERALL:                {'PASS' if overall_pass else 'FAIL'}")
        
        return overall_pass
    
    def cleanup(self):
        """Close connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()

def main():
    print("="*70)
    print(f"20 Hz PERFORMANCE TEST @ {BAUD_RATE} baud")
    print("="*70)
    
    tester = Hz20Tester(SERIAL_PORT, BAUD_RATE)
    
    if not tester.connect():
        print("Failed to connect. Check:")
        print("  1. Teensy is connected and programmed")
        print("  2. Serial port is correct (/dev/teensy)")
        print("  3. No other programs are using the port")
        return
    
    try:
        tester.run_test(TEST_DURATION)
        passed = tester.analyze_results()
        
        if passed:
            print(f"\n{'='*70}")
            print("SUCCESS: 20 Hz operation is working reliably!")
            print("You can use this baud rate in your production code.")
            print("="*70)
        else:
            print(f"\n{'='*70}")
            print("NEEDS IMPROVEMENT: Some timing issues detected.")
            print("Consider:")
            print("  1. Reducing target frequency")
            print("  2. Optimizing code on both sides")
            print("  3. Checking for USB interference")
            print("="*70)
            
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
