#!/usr/bin/env python3
"""
teensy_serial_bridge_test.py
Simplified version of the serial bridge for testing CMD communication
Shows all messages clearly for debugging
"""

import serial
import socket
import json
import time
import threading
import select
from datetime import datetime
from collections import defaultdict

# Configuration
SERIAL_PORT = '/dev/teensy'
BAUD_RATE = 921600
UDP_COMMAND_PORT = 6004

class TeensyTestBridge:
    def __init__(self):
        self.running = True
        self.ser = None
        self.command_sock = None
        
        # Statistics
        self.cmd_received_count = 0
        self.cmd_sent_count = 0
        self.teensy_messages = 0
        self.echo_count = 0
        
        # NEW: Message type counters for Teensy messages
        self.teensy_msg_types = defaultdict(int)
        
        self.setup()
    
    def setup(self):
        """Initialize serial and UDP connections"""
        # Serial connection
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            print(f"✅ Serial connected to {SERIAL_PORT} at {BAUD_RATE} baud")
        except serial.SerialException as e:
            print(f"❌ Failed to open serial port: {e}")
            raise
        
        # UDP socket for receiving commands
        try:
            self.command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.command_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.command_sock.bind(('', UDP_COMMAND_PORT))
            self.command_sock.setblocking(False)
            print(f"✅ UDP command listener on port {UDP_COMMAND_PORT}")
        except socket.error as e:
            print(f"❌ Failed to create UDP socket: {e}")
            raise
    
    def process_teensy_message(self, line):
        """Process messages from Teensy and display them"""
        self.teensy_messages += 1
        
        # Color code different message types and count by type
        if line.startswith("HEARTBEAT"):
            # print(f"💓 {line}")
            self.teensy_msg_types['HEARTBEAT'] += 1
        elif line.startswith("STATS"):
            print(f"📊 {line}")
            self.teensy_msg_types['STATS'] += 1
        elif line.startswith("ECHO"):
            self.echo_count += 1
            self.teensy_msg_types['ECHO'] += 1
            # print(f"🔄 {line}")
        elif line.startswith("WARNING"):
            print(f"⚠️  {line}")
            self.teensy_msg_types['WARNING'] += 1
        elif line.startswith("==="):
            print(f"📋 {line}")
            self.teensy_msg_types['SECTION'] += 1
        elif line.startswith("INFO"):
            print(f"ℹ️  {line}")
            self.teensy_msg_types['INFO'] += 1
        else:
            print(f"📥 TEENSY: {line}")
            self.teensy_msg_types['OTHER'] += 1
    
    def send_cmd_to_teensy(self, linear_x, angular_z):
        """Send CMD message to Teensy"""
        command = f"CMD,{linear_x:.4f},{angular_z:.4f}\n"
        self.ser.write(command.encode('utf-8'))
        self.cmd_sent_count += 1
        
        # # Show every 20th command sent
        # if self.cmd_sent_count % 20 == 1:
        #     print(f"📤 Sent CMD #{self.cmd_sent_count}: linear_x={linear_x:.3f}, angular_z={angular_z:.3f}")
    
    def listen_for_udp_commands(self):
        """Thread to listen for UDP commands"""
        print("🎧 UDP listener thread started")
        
        while self.running:
            try:
                # Check for UDP data
                ready = select.select([self.command_sock], [], [], 0.1)
                if ready[0]:
                    # Read all available packets
                    while True:
                        try:
                            data, addr = self.command_sock.recvfrom(1024)
                            command = json.loads(data.decode())
                            linear_x = command.get('linear_x', 0.0)
                            angular_z = command.get('angular_z', 0.0)
                            
                            self.cmd_received_count += 1
                            self.send_cmd_to_teensy(linear_x, angular_z)
                            
                        except socket.error:
                            break  # No more data
                        except json.JSONDecodeError as e:
                            print(f"❌ Invalid JSON: {e}")
                
            except Exception as e:
                if self.running:
                    print(f"❌ UDP listener error: {e}")
        
        print("👋 UDP listener thread stopped")
    
    def print_statistics(self, elapsed):
        """Print statistics periodically"""
        # Compute rates based on elapsed time
        cmd_received_rate = self.cmd_received_count / elapsed if elapsed > 0 else 0
        cmd_sent_rate = self.cmd_sent_count / elapsed if elapsed > 0 else 0
        teensy_rate = self.teensy_messages / elapsed if elapsed > 0 else 0
        echo_rate = self.echo_count / elapsed if elapsed > 0 else 0
        
        print(f"\n{'='*70}")
        print(f"📈 BRIDGE STATISTICS (Last {elapsed:.1f}s)")
        print(f"{'='*70}")
        
        # Section a: Messages received from port 6004
        print(f"--- UDP 6004 Reception ---")
        print(f"  Messages received: {self.cmd_received_count} ({cmd_received_rate:.1f} Hz)")
        
        # Section b: Messages sent to teensy
        print(f"\n--- Serial to Teensy ---")
        print(f"  Commands sent: {self.cmd_sent_count} ({cmd_sent_rate:.1f} Hz)")
        
        # Section c: Messages received from teensy
        print(f"\n--- Serial from Teensy ---")
        print(f"  Messages received: {self.teensy_messages} ({teensy_rate:.1f} Hz)")
        print(f"  Echo confirmations: {self.echo_count} ({echo_rate:.1f} Hz)")
        print(f"  Pending echoes: {self.cmd_sent_count - self.echo_count}")
        
        # NEW: Teensy message breakdown by type
        print(f"\n  Teensy Message Breakdown:")
        for msg_type in sorted(self.teensy_msg_types.keys()):
            count = self.teensy_msg_types[msg_type]
            rate = count / elapsed if elapsed > 0 else 0
            print(f"    {msg_type:<12}: {count:4d} ({rate:.1f} Hz)")
        
        print(f"{'='*70}\n")
        
        # Reset all counters to zero after reporting
        self.cmd_received_count = 0
        self.cmd_sent_count = 0
        self.teensy_messages = 0
        self.echo_count = 0
        self.teensy_msg_types.clear()
    
    def run(self):
        """Main loop"""
        print("\n🚀 Teensy Test Bridge Starting...")
        print("="*50)
        
        # Start UDP listener thread
        udp_thread = threading.Thread(target=self.listen_for_udp_commands, daemon=True)
        udp_thread.start()
        
        last_stats_time = time.time()
        stats_interval = 20.0  # Print stats every 20 seconds
        
        try:
            while True:
                # Read from serial
                if self.ser.in_waiting:
                    try:
                        line = self.ser.readline().decode('utf-8').strip()
                        if line:
                            self.process_teensy_message(line)
                    except UnicodeDecodeError:
                        pass
                
                # Print statistics periodically
                current_time = time.time()
                if current_time - last_stats_time >= stats_interval:
                    elapsed = current_time - last_stats_time
                    self.print_statistics(elapsed)
                    last_stats_time = current_time
                
                time.sleep(0.001)  # Small delay to prevent CPU spinning
                
        except KeyboardInterrupt:
            print("\n\n🛑 Shutting down...")
        finally:
            # Final statistics (cumulative since last reset)
            elapsed = time.time() - last_stats_time
            if elapsed > 0:  # Only if some time passed
                self.print_statistics(elapsed)
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        
        print("\n📊 FINAL STATISTICS")
        # Note: Final stats already printed in run() finally block
        
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✅ Serial port closed")
        
        if self.command_sock:
            self.command_sock.close()
            print("✅ UDP socket closed")

def main():
    try:
        bridge = TeensyTestBridge()
        bridge.run()
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        raise

if __name__ == "__main__":
    main()