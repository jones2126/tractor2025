#!/usr/bin/env python3
"""
test_cmd_vel_sender.py
======================
Test script to send cmd_vel commands to the Teensy bridge.

Usage:
  python3 test_cmd_vel_sender.py               # Interactive mode
  python3 test_cmd_vel_sender.py --pattern     # Automated test patterns
  python3 test_cmd_vel_sender.py --monitor     # Monitor echoes only
"""

import socket
import json
import time
import argparse
import sys

UDP_COMMAND_PORT = 6004
UDP_STATUS_PORT = 6003
BROADCAST_IP = '255.255.255.255'

class CmdVelTester:
    def __init__(self):
        """Initialize command sender and status monitor"""
        # Socket for sending commands
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        # Socket for monitoring status
        self.status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.status_sock.bind(('', UDP_STATUS_PORT))
        self.status_sock.settimeout(1.0)
        
        self.command_count = 0
        print(f"Initialized - Sending to port {UDP_COMMAND_PORT}, Monitoring port {UDP_STATUS_PORT}")
    
    def send_cmd_vel(self, linear_x, angular_z):
        """Send a cmd_vel command"""
        command = {
            'linear_x': linear_x,
            'angular_z': angular_z,
            'timestamp': time.time()
        }
        
        json_data = json.dumps(command)
        self.cmd_sock.sendto(json_data.encode(), (BROADCAST_IP, UDP_COMMAND_PORT))
        
        self.command_count += 1
        print(f"[{self.command_count:4d}] Sent: linear_x={linear_x:+.3f}, angular_z={angular_z:+.3f}")
        
        return command
    
    def monitor_status(self, timeout=2.0):
        """Monitor for status broadcasts to verify command receipt"""
        print(f"\nMonitoring for {timeout} seconds...")
        start_time = time.time()
        last_echo_count = 0
        
        while time.time() - start_time < timeout:
            try:
                data, addr = self.status_sock.recvfrom(4096)
                status = json.loads(data.decode())
                
                # Check cmd_vel section
                if 'cmd_vel' in status:
                    cmd_info = status['cmd_vel']
                    echo_count = cmd_info.get('commands_echoed', 0)
                    
                    # New echo received
                    if echo_count > last_echo_count:
                        print(f"✓ Bridge reports: {echo_count} commands echoed by Teensy")
                        print(f"  Last cmd: linear_x={cmd_info['last_linear_x']:.3f}, "
                              f"angular_z={cmd_info['last_angular_z']:.3f}")
                        last_echo_count = echo_count
                    
            except socket.timeout:
                continue
            except json.JSONDecodeError:
                continue
        
        if last_echo_count == 0:
            print("⚠ No command echoes detected")
        
        return last_echo_count
    
    def interactive_mode(self):
        """Interactive command entry"""
        print("\n" + "="*60)
        print("Interactive cmd_vel Sender")
        print("="*60)
        print("Enter commands as: <linear_x> <angular_z>")
        print("Examples:")
        print("  0.5 0.0    - Move forward at 0.5 m/s")
        print("  0.0 0.5    - Turn left at 0.5 rad/s")
        print("  0.5 -0.3   - Forward and turn right")
        print("  stop       - Send stop command (0.0, 0.0)")
        print("  quit       - Exit program")
        print("="*60 + "\n")
        
        while True:
            try:
                user_input = input("cmd_vel> ").strip().lower()
                
                if user_input == 'quit' or user_input == 'q':
                    print("Exiting...")
                    break
                
                if user_input == 'stop' or user_input == 's':
                    self.send_cmd_vel(0.0, 0.0)
                    self.monitor_status(timeout=1.0)
                    continue
                
                if user_input == 'monitor' or user_input == 'm':
                    self.monitor_status(timeout=5.0)
                    continue
                
                # Parse linear_x and angular_z
                parts = user_input.split()
                if len(parts) != 2:
                    print("Error: Enter two numbers (linear_x angular_z)")
                    continue
                
                try:
                    linear_x = float(parts[0])
                    angular_z = float(parts[1])
                    
                    self.send_cmd_vel(linear_x, angular_z)
                    self.monitor_status(timeout=1.0)
                    
                except ValueError:
                    print("Error: Invalid numbers")
                
            except KeyboardInterrupt:
                print("\nExiting...")
                break
    
    def pattern_mode(self):
        """Run automated test patterns"""
        print("\n" + "="*60)
        print("Automated Test Pattern Mode")
        print("="*60)
        
        test_patterns = [
            # (name, linear_x, angular_z, duration)
            ("Stop", 0.0, 0.0, 2),
            ("Forward slow", 0.2, 0.0, 3),
            ("Stop", 0.0, 0.0, 2),
            ("Forward medium", 0.5, 0.0, 3),
            ("Stop", 0.0, 0.0, 2),
            ("Turn left", 0.0, 0.5, 3),
            ("Stop", 0.0, 0.0, 2),
            ("Turn right", 0.0, -0.5, 3),
            ("Stop", 0.0, 0.0, 2),
            ("Forward + left", 0.3, 0.3, 3),
            ("Stop", 0.0, 0.0, 2),
            ("Forward + right", 0.3, -0.3, 3),
            ("Stop", 0.0, 0.0, 2),
            ("Reverse slow", -0.2, 0.0, 3),
            ("Final stop", 0.0, 0.0, 2),
        ]
        
        print(f"Running {len(test_patterns)} test patterns...\n")
        
        total_echoes = 0
        
        for i, (name, linear_x, angular_z, duration) in enumerate(test_patterns, 1):
            print(f"\n[Pattern {i}/{len(test_patterns)}] {name}")
            print(f"  Command: linear_x={linear_x:+.1f}, angular_z={angular_z:+.1f}")
            print(f"  Duration: {duration}s")
            
            # Send command repeatedly during duration
            start_time = time.time()
            commands_sent = 0
            
            while time.time() - start_time < duration:
                self.send_cmd_vel(linear_x, angular_z)
                commands_sent += 1
                time.sleep(0.1)  # 10 Hz
            
            # Check for echoes
            echoes = self.monitor_status(timeout=1.0)
            total_echoes += echoes
            
            print(f"  Sent {commands_sent} commands")
        
        print("\n" + "="*60)
        print(f"Test Complete!")
        print(f"Total commands sent: {self.command_count}")
        print(f"Total echoes detected: {total_echoes}")
        
        if total_echoes > 0:
            print("✓ SUCCESS: Teensy is receiving and echoing commands")
        else:
            print("⚠ WARNING: No echoes detected - check Teensy code")
        print("="*60)
    
    def monitor_only_mode(self):
        """Just monitor status without sending"""
        print("\n" + "="*60)
        print("Monitor Mode - Listening for status broadcasts...")
        print("Press Ctrl+C to exit")
        print("="*60 + "\n")
        
        last_echo_count = 0
        
        try:
            while True:
                try:
                    data, addr = self.status_sock.recvfrom(4096)
                    status = json.loads(data.decode())
                    
                    if 'cmd_vel' in status:
                        cmd_info = status['cmd_vel']
                        echo_count = cmd_info.get('commands_echoed', 0)
                        
                        print(f"[{time.strftime('%H:%M:%S')}] "
                              f"Echoes: {echo_count}, "
                              f"Last: linear_x={cmd_info['last_linear_x']:+.3f}, "
                              f"angular_z={cmd_info['last_angular_z']:+.3f}, "
                              f"age={cmd_info['age']:.2f}s")
                        
                        if echo_count > last_echo_count:
                            print(f"  → NEW ECHO detected!")
                            last_echo_count = echo_count
                
                except socket.timeout:
                    continue
                except json.JSONDecodeError:
                    continue
        
        except KeyboardInterrupt:
            print("\nExiting monitor mode...")
    
    def quick_test(self):
        """Quick connectivity test"""
        print("\n" + "="*60)
        print("Quick Connectivity Test")
        print("="*60)
        
        print("\n1. Sending test command...")
        self.send_cmd_vel(0.5, 0.0)
        
        print("\n2. Waiting for echo (5 seconds)...")
        echoes = self.monitor_status(timeout=5.0)
        
        if echoes > 0:
            print("\n✓ SUCCESS: Communication working!")
            print("  - Bridge received UDP command")
            print("  - Bridge sent to Teensy via serial")
            print("  - Teensy echoed command back")
        else:
            print("\n⚠ PROBLEM: No echo detected")
            print("  Possible issues:")
            print("  - Bridge not running")
            print("  - Teensy not connected")
            print("  - Teensy code not echoing commands")
            print("  Run with --monitor to check for any activity")
        
        print("="*60)
    
    def cleanup(self):
        """Close sockets"""
        self.cmd_sock.close()
        self.status_sock.close()


def main():
    parser = argparse.ArgumentParser(description='Test cmd_vel sender for Teensy bridge')
    parser.add_argument('--pattern', action='store_true', 
                       help='Run automated test patterns')
    parser.add_argument('--monitor', action='store_true',
                       help='Monitor mode only (no sending)')
    parser.add_argument('--quick', action='store_true',
                       help='Quick connectivity test')
    
    args = parser.parse_args()
    
    try:
        tester = CmdVelTester()
        
        if args.pattern:
            tester.pattern_mode()
        elif args.monitor:
            tester.monitor_only_mode()
        elif args.quick:
            tester.quick_test()
        else:
            # Default to interactive mode
            tester.interactive_mode()
        
        tester.cleanup()
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
