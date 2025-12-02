#!/usr/bin/env python3
"""
cmd_vel_sender.py
Sends test CMD velocity messages at 20Hz to the Teensy serial bridge
via UDP port 6004
"""

import socket
import json
import time
import math
import argparse
import signal
import sys

class CmdVelSender:
    def __init__(self, target_ip='127.0.0.1', target_port=6004, rate_hz=20):
        """
        Initialize the CMD velocity sender
        
        Args:
            target_ip: IP address to send UDP packets to
            target_port: UDP port (default 6004 for teensy_serial_bridge)
            rate_hz: Target send rate in Hz
        """
        self.target_ip = target_ip
        self.target_port = target_port
        self.rate_hz = rate_hz
        self.period = 1.0 / rate_hz
        
        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Statistics
        self.messages_sent = 0
        self.start_time = time.time()
        self.last_stats_time = time.time()
        self.last_stats_count = 0
        
        # Control for graceful shutdown
        self.running = True
        
    def generate_test_values(self, t):
        """
        Generate test values that vary over time
        
        Args:
            t: Time in seconds since start
            
        Returns:
            tuple: (linear_x, angular_z) values
        """
        # Create sinusoidal test patterns
        linear_x = 0.5 * math.sin(2 * math.pi * 0.1 * t)  # 0.1 Hz oscillation
        angular_z = 0.3 * math.cos(2 * math.pi * 0.2 * t)  # 0.2 Hz oscillation
        
        return linear_x, angular_z
    
    def send_cmd_vel(self, linear_x, angular_z):
        """
        Send a CMD velocity message via UDP
        
        Args:
            linear_x: Linear velocity (-1.0 to 1.0)
            angular_z: Angular velocity (-1.0 to 1.0)
        """
        # Create the command dictionary
        cmd = {
            'linear_x': linear_x,
            'angular_z': angular_z,
            'timestamp': time.time()
        }
        
        # Convert to JSON and send
        message = json.dumps(cmd).encode('utf-8')
        self.sock.sendto(message, (self.target_ip, self.target_port))
        self.messages_sent += 1
        
    def print_statistics(self):
        """Print sending statistics every 5 seconds"""
        current_time = time.time()
        if current_time - self.last_stats_time >= 5.0:
            elapsed = current_time - self.last_stats_time
            interval_messages = self.messages_sent - self.last_stats_count
            actual_rate = interval_messages / elapsed
            
            total_elapsed = current_time - self.start_time
            overall_rate = self.messages_sent / total_elapsed
            
            print(f"\n=== SENDER STATISTICS ===")
            print(f"Messages sent: {self.messages_sent}")
            print(f"Target rate: {self.rate_hz:.1f} Hz")
            print(f"Actual rate (5s): {actual_rate:.1f} Hz")
            print(f"Overall rate: {overall_rate:.1f} Hz")
            print(f"Running time: {total_elapsed:.1f} seconds")
            print(f"=========================\n")
            
            self.last_stats_time = current_time
            self.last_stats_count = self.messages_sent
    
    def run(self):
        """Main sending loop"""
        print(f"Starting CMD velocity sender")
        print(f"Target: {self.target_ip}:{self.target_port}")
        print(f"Rate: {self.rate_hz} Hz")
        print(f"Press Ctrl+C to stop\n")
        
        next_send_time = time.time()
        
        try:
            while self.running:
                current_time = time.time()
                
                # Send message if it's time
                if current_time >= next_send_time:
                    # Generate test values
                    elapsed = current_time - self.start_time
                    linear_x, angular_z = self.generate_test_values(elapsed)
                    
                    # Send the command
                    self.send_cmd_vel(linear_x, angular_z)
                    
                    # Print every 20th message for visibility
                    if self.messages_sent % 20 == 0:
                        print(f"Sent #{self.messages_sent}: linear_x={linear_x:6.3f}, angular_z={angular_z:6.3f}")
                    
                    # Schedule next send
                    next_send_time += self.period
                    
                    # Handle timing drift
                    if next_send_time < current_time:
                        next_send_time = current_time + self.period
                
                # Print statistics periodically
                self.print_statistics()
                
                # Sleep until next send time
                sleep_time = next_send_time - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
        except KeyboardInterrupt:
            print("\n\nShutting down...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        
        # Print final statistics
        total_elapsed = time.time() - self.start_time
        overall_rate = self.messages_sent / total_elapsed if total_elapsed > 0 else 0
        
        print(f"\n=== FINAL STATISTICS ===")
        print(f"Total messages sent: {self.messages_sent}")
        print(f"Total time: {total_elapsed:.1f} seconds")
        print(f"Average rate: {overall_rate:.1f} Hz")
        print(f"========================")
        
        self.sock.close()

def main():
    parser = argparse.ArgumentParser(description='Send test CMD velocity messages')
    parser.add_argument('--ip', default='127.0.0.1',
                        help='Target IP address (default: 127.0.0.1 for local)')
    parser.add_argument('--port', type=int, default=6004,
                        help='Target UDP port (default: 6004)')
    parser.add_argument('--rate', type=float, default=20.0,
                        help='Send rate in Hz (default: 20.0)')
    parser.add_argument('--pattern', default='sine',
                        choices=['sine', 'constant', 'step', 'random'],
                        help='Test pattern to use (default: sine)')
    
    args = parser.parse_args()
    
    # Create and run the sender
    sender = CmdVelSender(
        target_ip=args.ip,
        target_port=args.port,
        rate_hz=args.rate
    )
    
    # Handle different test patterns
    if args.pattern == 'constant':
        # Override the generate_test_values method for constant values
        sender.generate_test_values = lambda t: (0.5, 0.0)
    elif args.pattern == 'step':
        # Step function pattern
        sender.generate_test_values = lambda t: (
            0.5 if int(t/5) % 2 == 0 else -0.5,
            0.3 if int(t/3) % 2 == 0 else -0.3
        )
    elif args.pattern == 'random':
        import random
        sender.generate_test_values = lambda t: (
            random.uniform(-1.0, 1.0),
            random.uniform(-1.0, 1.0)
        )
    
    # Install signal handler for clean shutdown
    signal.signal(signal.SIGINT, lambda s, f: sender.cleanup())
    
    sender.run()

if __name__ == "__main__":
    main()
