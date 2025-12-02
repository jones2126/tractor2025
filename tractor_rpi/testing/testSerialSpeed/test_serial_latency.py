#!/usr/bin/env python3
"""
test_serial_latency.py
Measures serial communication latency and reliability at different baud rates
"""

import serial
import time
import sys
from collections import deque

# Test configuration
SERIAL_PORT = '/dev/teensy'
TEST_BAUD_RATES = [115200, 230400, 460800, 921600, 1000000, 2000000]
TEST_DURATION = 10  # seconds per baud rate

class SerialLatencyTester:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        
        self.heartbeats_received = 0
        self.commands_sent = 0
        self.echoes_received = 0
        self.latencies = deque(maxlen=1000)
        self.heartbeat_intervals = deque(maxlen=100)
        self.last_heartbeat_time = None
        
    def connect(self):
        """Open serial connection"""
        try:
            self.ser = serial.Serial(
                self.port, 
                self.baud_rate,
                timeout=0.1,
                write_timeout=0.1
            )
            time.sleep(2)  # Wait for Teensy to restart
            
            # Read startup messages
            start = time.time()
            while time.time() - start < 3:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"  Startup: {line}")
                        if "TEENSY_READY" in line:
                            return True
            return True
        except Exception as e:
            print(f"  Connection failed: {e}")
            return False
    
    def send_command(self, linear_x, angular_z):
        """Send a command and return send timestamp"""
        try:
            command = f"CMD,{linear_x:.4f},{angular_z:.4f}\n"
            self.ser.write(command.encode('utf-8'))
            self.ser.flush()
            self.commands_sent += 1
            return time.time()
        except Exception as e:
            print(f"  Send error: {e}")
            return None
    
    def read_messages(self):
        """Read and process all available messages"""
        while self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                
                parts = line.split(',')
                
                if parts[0] == "ECHO":
                    # ECHO,timestamp,count,original_command
                    self.echoes_received += 1
                    if hasattr(self, 'last_command_time') and self.last_command_time:
                        latency = (time.time() - self.last_command_time) * 1000  # ms
                        self.latencies.append(latency)
                
                elif parts[0] == "HEARTBEAT":
                    # HEARTBEAT,timestamp,count
                    self.heartbeats_received += 1
                    current_time = time.time()
                    if self.last_heartbeat_time:
                        interval = (current_time - self.last_heartbeat_time) * 1000  # ms
                        self.heartbeat_intervals.append(interval)
                    self.last_heartbeat_time = current_time
                
            except Exception as e:
                pass  # Skip malformed messages
    
    def run_test(self, duration):
        """Run test for specified duration"""
        print(f"\n  Testing at {self.baud_rate} baud for {duration} seconds...")
        
        start_time = time.time()
        last_command_time = time.time()
        command_interval = 0.05  # 20 Hz
        
        while time.time() - start_time < duration:
            current_time = time.time()
            
            # Send command at 20 Hz
            if current_time - last_command_time >= command_interval:
                linear_x = 0.5
                angular_z = 0.1
                self.last_command_time = self.send_command(linear_x, angular_z)
                last_command_time = current_time
            
            # Read all incoming messages
            self.read_messages()
            
            time.sleep(0.001)  # Small sleep to prevent CPU spinning
        
        # Final read
        time.sleep(0.1)
        self.read_messages()
    
    def print_results(self):
        """Print test results"""
        print(f"\n  Results for {self.baud_rate} baud:")
        print(f"    Commands sent: {self.commands_sent}")
        print(f"    Echoes received: {self.echoes_received} ({100*self.echoes_received/max(1,self.commands_sent):.1f}%)")
        print(f"    Heartbeats received: {self.heartbeats_received}")
        
        if self.latencies:
            avg_latency = sum(self.latencies) / len(self.latencies)
            min_latency = min(self.latencies)
            max_latency = max(self.latencies)
            print(f"    Latency: avg={avg_latency:.2f}ms, min={min_latency:.2f}ms, max={max_latency:.2f}ms")
        
        if self.heartbeat_intervals:
            avg_interval = sum(self.heartbeat_intervals) / len(self.heartbeat_intervals)
            min_interval = min(self.heartbeat_intervals)
            max_interval = max(self.heartbeat_intervals)
            actual_hz = 1000.0 / avg_interval if avg_interval > 0 else 0
            print(f"    Heartbeat interval: avg={avg_interval:.2f}ms (={actual_hz:.1f}Hz), "
                  f"min={min_interval:.2f}ms, max={max_interval:.2f}ms")
        
        # Score based on completeness and latency
        completeness = self.echoes_received / max(1, self.commands_sent)
        avg_lat = sum(self.latencies) / len(self.latencies) if self.latencies else 999
        score = completeness * 100 - avg_lat
        print(f"    Score: {score:.1f} (higher is better)")
        
        return score
    
    def cleanup(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()

def main():
    print("=" * 60)
    print("Serial Communication Latency Tester")
    print("=" * 60)
    
    results = {}
    
    for baud_rate in TEST_BAUD_RATES:
        print(f"\n{'='*60}")
        print(f"Testing baud rate: {baud_rate}")
        print('='*60)
        
        tester = SerialLatencyTester(SERIAL_PORT, baud_rate)
        
        if not tester.connect():
            print(f"  Skipping {baud_rate} - connection failed")
            continue
        
        try:
            tester.run_test(TEST_DURATION)
            score = tester.print_results()
            results[baud_rate] = score
        except KeyboardInterrupt:
            print("\n  Test interrupted by user")
            break
        except Exception as e:
            print(f"  Test failed: {e}")
        finally:
            tester.cleanup()
        
        time.sleep(2)  # Wait between tests
    
    # Print summary
    print(f"\n{'='*60}")
    print("SUMMARY")
    print('='*60)
    for baud_rate, score in sorted(results.items(), key=lambda x: x[1], reverse=True):
        print(f"  {baud_rate:>10} baud: score={score:>6.1f}")
    
    if results:
        best_baud = max(results.items(), key=lambda x: x[1])[0]
        print(f"\nRecommended baud rate: {best_baud}")

if __name__ == "__main__":
    main()
