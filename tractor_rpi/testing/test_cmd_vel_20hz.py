#!/usr/bin/env python3
"""
test_cmd_vel_20hz.py v2.0
=========================
Test cmd_vel communication at various rates with improved UDP reliability.
"""

import socket
import json
import time
import argparse
import sys
import math
from datetime import datetime

UDP_COMMAND_PORT = 6004
UDP_STATUS_PORT = 6003
BROADCAST_IP = '255.255.255.255'

class HighRateTester:
    def __init__(self):
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        self.status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.status_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
        self.status_sock.bind(('', UDP_STATUS_PORT))
        self.status_sock.settimeout(0.1)
        
        self.command_count = 0
        self.start_time = None
        self.broadcast_count = 0
        self.last_echo_count = 0
        
        print(f"High-Rate Tester v2.0 initialized")
        print(f"Sending to port {UDP_COMMAND_PORT}")
        print(f"Monitoring port {UDP_STATUS_PORT}")
    
    def send_cmd_vel(self, linear_x, angular_z, verbose=False):
        command = {
            'linear_x': linear_x,
            'angular_z': angular_z,
            'timestamp': time.time()
        }
        
        json_data = json.dumps(command)
        self.cmd_sock.sendto(json_data.encode(), (BROADCAST_IP, UDP_COMMAND_PORT))
        
        self.command_count += 1
        
        if verbose and self.command_count % 100 == 0:
            print(f"[{self.command_count:5d}] Sent: linear_x={linear_x:+.3f}, "
                  f"angular_z={angular_z:+.3f}")
        
        return command
    
    def get_echo_count(self, timeout=2.0, attempts=5):
        start = time.time()
        echo_counts = []
        broadcasts_received = 0
        
        print(f"  Reading echo count (timeout={timeout}s, attempts={attempts})...")
        
        while time.time() - start < timeout and len(echo_counts) < attempts:
            try:
                data, addr = self.status_sock.recvfrom(4096)
                status = json.loads(data.decode())
                broadcasts_received += 1
                
                if 'cmd_vel' in status:
                    echo_count = status['cmd_vel'].get('commands_echoed', 0)
                    echo_counts.append(echo_count)
                    print(f"    Broadcast #{broadcasts_received}: echo_count={echo_count}")
                    
            except socket.timeout:
                continue
            except json.JSONDecodeError as e:
                print(f"    JSON decode error: {e}")
                continue
            except Exception as e:
                print(f"    Unexpected error: {e}")
                continue
        
        if not echo_counts:
            print(f"  ⚠ WARNING: No broadcasts received in {timeout}s!")
            return self.last_echo_count, 0
        
        final_echo_count = max(echo_counts)
        self.last_echo_count = final_echo_count
        
        print(f"  ✓ Received {broadcasts_received} broadcasts, echo_count={final_echo_count}")
        
        return final_echo_count, broadcasts_received
    
    def diagnose_connection(self, duration=10):
        print("\n" + "="*70)
        print("UDP Broadcast Diagnostic Mode")
        print("="*70)
        print(f"Listening for {duration} seconds...")
        print(f"Broadcasts expected: ~{duration * 5} (at 5 Hz)")
        print("-"*70)
        
        broadcasts = []
        start = time.time()
        
        try:
            while time.time() - start < duration:
                try:
                    data, addr = self.status_sock.recvfrom(4096)
                    recv_time = time.time()
                    
                    try:
                        status = json.loads(data.decode())
                        broadcasts.append({
                            'time': recv_time,
                            'size': len(data),
                            'valid': True,
                            'has_cmd_vel': 'cmd_vel' in status,
                            'echo_count': status.get('cmd_vel', {}).get('commands_echoed', 0) if 'cmd_vel' in status else 0
                        })
                        
                        if len(broadcasts) % 5 == 0:
                            print(f"  [{len(broadcasts):3d}] Received broadcast from {addr}, "
                                  f"size={len(data)} bytes, "
                                  f"echo_count={broadcasts[-1]['echo_count']}")
                    
                    except json.JSONDecodeError:
                        broadcasts.append({
                            'time': recv_time,
                            'size': len(data),
                            'valid': False,
                            'has_cmd_vel': False,
                            'echo_count': 0
                        })
                
                except socket.timeout:
                    continue
        
        except KeyboardInterrupt:
            print("\n  Interrupted by user")
        
        actual_duration = time.time() - start
        valid_broadcasts = sum(1 for b in broadcasts if b['valid'])
        with_cmd_vel = sum(1 for b in broadcasts if b['has_cmd_vel'])
        
        print("\n" + "="*70)
        print("Diagnostic Results")
        print("="*70)
        print(f"Duration:              {actual_duration:.1f} seconds")
        print(f"Broadcasts received:   {len(broadcasts)}")
        print(f"Valid JSON:            {valid_broadcasts}")
        print(f"With cmd_vel data:     {with_cmd_vel}")
        print(f"Expected broadcasts:   ~{int(actual_duration * 5)}")
        print(f"Reception rate:        {len(broadcasts) / actual_duration:.1f} Hz")
        print("-"*70)
        
        if len(broadcasts) > 1:
            intervals = [broadcasts[i]['time'] - broadcasts[i-1]['time'] 
                        for i in range(1, len(broadcasts))]
            avg_interval = sum(intervals) / len(intervals)
            print(f"Average interval:      {avg_interval*1000:.1f} ms")
            print(f"Min interval:          {min(intervals)*1000:.1f} ms")
            print(f"Max interval:          {max(intervals)*1000:.1f} ms")
        
        print("="*70)
        
        if len(broadcasts) >= actual_duration * 4.5:
            print("✓ EXCELLENT: Receiving broadcasts reliably")
        elif len(broadcasts) >= actual_duration * 2.5:
            print("⚠ WARNING: Some packet loss, but usable")
        elif len(broadcasts) > 0:
            print("✗ POOR: Significant packet loss - check network")
        else:
            print("✗ FAIL: No broadcasts received - bridge may not be running")
        
        print("="*70)
    
    def baseline_20hz_test(self, duration=30):
        print("\n" + "="*70)
        print("20 Hz Baseline Test")
        print("="*70)
        print(f"Duration: {duration} seconds")
        print(f"Target rate: 20 Hz (50ms interval)")
        print(f"Expected commands: {duration * 20}")
        print("-"*70)
        
        print("\nGetting baseline echo count...")
        starting_echoes, broadcasts_rx = self.get_echo_count(timeout=3.0, attempts=10)
        
        if broadcasts_rx == 0:
            print("⚠ WARNING: Cannot read echo count - continuing anyway")
        
        initial_commands = self.command_count
        
        patterns = [
            ("Forward 0.5 m/s", 0.5, 0.0),
            ("Forward 0.3 m/s", 0.3, 0.0),
            ("Turn left", 0.0, 0.3),
            ("Turn right", 0.0, -0.3),
            ("Arc left", 0.4, 0.2),
            ("Arc right", 0.4, -0.2),
        ]
        
        pattern_duration = duration / len(patterns)
        
        print(f"\nRunning {len(patterns)} patterns, {pattern_duration:.1f}s each...")
        print("Starting in 2 seconds...")
        time.sleep(2)
        
        test_start = time.time()
        pattern_start = test_start
        pattern_idx = 0
        
        timing_errors = []
        max_interval = 0
        min_interval = 1.0
        
        last_send_time = time.time()
        
        linear_x, angular_z = patterns[pattern_idx][1], patterns[pattern_idx][2]
        print(f"\n[Pattern 1/{len(patterns)}] {patterns[pattern_idx][0]}")
        
        while time.time() - test_start < duration:
            loop_start = time.time()
            
            if time.time() - pattern_start >= pattern_duration and pattern_idx < len(patterns) - 1:
                pattern_idx += 1
                pattern_start = time.time()
                linear_x, angular_z = patterns[pattern_idx][1], patterns[pattern_idx][2]
                print(f"[Pattern {pattern_idx+1}/{len(patterns)}] {patterns[pattern_idx][0]}")
            
            self.send_cmd_vel(linear_x, angular_z, verbose=False)
            
            interval = time.time() - last_send_time
            max_interval = max(max_interval, interval)
            min_interval = min(min_interval, interval)
            last_send_time = time.time()
            
            elapsed = time.time() - loop_start
            sleep_time = 0.050 - elapsed
            
            if sleep_time < 0:
                timing_errors.append(abs(sleep_time))
            else:
                time.sleep(sleep_time)
        
        test_end = time.time()
        actual_duration = test_end - test_start
        
        print("\n" + "-"*70)
        print("Test complete. Waiting 2s for final echoes...")
        time.sleep(2)
        ending_echoes, broadcasts_rx = self.get_echo_count(timeout=3.0, attempts=10)
        
        commands_sent = self.command_count - initial_commands
        echoes_received = ending_echoes - starting_echoes
        actual_rate = commands_sent / actual_duration
        
        print("\n" + "="*70)
        print("20 Hz Test Results")
        print("="*70)
        print(f"Duration:           {actual_duration:.2f} seconds")
        print(f"Commands sent:      {commands_sent}")
        print(f"Commands echoed:    {echoes_received}")
        print(f"Lost commands:      {commands_sent - echoes_received}")
        print(f"Success rate:       {(echoes_received/commands_sent)*100 if commands_sent > 0 else 0:.2f}%")
        print(f"Actual send rate:   {actual_rate:.2f} Hz")
        print(f"Target rate:        20.00 Hz")
        print(f"Rate accuracy:      {(actual_rate/20.0)*100:.1f}%")
        print("-"*70)
        print(f"Timing statistics:")
        print(f"  Min interval:     {min_interval*1000:.2f} ms")
        print(f"  Max interval:     {max_interval*1000:.2f} ms")
        print(f"  Target interval:  50.00 ms")
        if timing_errors:
            print(f"  Timing overruns:  {len(timing_errors)} ({len(timing_errors)/commands_sent*100:.1f}%)")
            print(f"  Max overrun:      {max(timing_errors)*1000:.2f} ms")
        else:
            print(f"  Timing overruns:  0 (Perfect!)")
        
        print("="*70)
        if broadcasts_rx == 0:
            print("⚠ UNKNOWN: Could not verify - echo count unavailable")
            print("  Run with --diagnose to troubleshoot UDP broadcasts")
        elif echoes_received == commands_sent:
            print("✓ PERFECT: 100% success rate, zero lost commands!")
        elif echoes_received >= commands_sent * 0.99:
            print("✓ EXCELLENT: >99% success rate")
        elif echoes_received >= commands_sent * 0.95:
            print("✓ GOOD: >95% success rate")
        elif echoes_received >= commands_sent * 0.90:
            print("⚠ WARNING: 90-95% success rate - some commands lost")
        else:
            print("✗ FAIL: <90% success rate - significant command loss")
        print("="*70)
    
    def progressive_stress_test(self):
        print("\n" + "="*70)
        print("Progressive Stress Test")
        print("="*70)
        print("Testing rates: 5 Hz, 10 Hz, 20 Hz, 30 Hz")
        print("Each test runs for 10 seconds")
        print("="*70)
        
        test_rates = [
            (5, 0.200, "5 Hz - Low rate baseline"),
            (10, 0.100, "10 Hz - ROS standard"),
            (20, 0.050, "20 Hz - F9P GPS rate"),
            (30, 0.033, "30 Hz - High-rate navigation"),
        ]
        
        results = {}
        
        for rate_hz, interval, description in test_rates:
            print(f"\n>>> Testing {description}...")
            
            starting_echoes, _ = self.get_echo_count(timeout=2.0, attempts=5)
            initial_commands = self.command_count
            
            test_start = time.time()
            duration = 10.0
            
            while time.time() - test_start < duration:
                self.send_cmd_vel(0.5, 0.0, verbose=False)
                time.sleep(interval)
            
            time.sleep(2)
            ending_echoes, broadcasts_rx = self.get_echo_count(timeout=2.0, attempts=5)
            
            sent = self.command_count - initial_commands
            echoed = ending_echoes - starting_echoes
            success = (echoed / sent) * 100 if sent > 0 else 0
            
            results[rate_hz] = {
                'sent': sent,
                'echoed': echoed,
                'success': success,
                'broadcasts': broadcasts_rx
            }
            
            print(f"  Sent: {sent}, Echoed: {echoed}, Success: {success:.1f}%, "
                  f"Broadcasts: {broadcasts_rx}")
            
            time.sleep(2)
        
        print("\n" + "="*70)
        print("Stress Test Summary")
        print("="*70)
        print(f"{'Rate':<10} {'Sent':<10} {'Echoed':<10} {'Lost':<10} {'Success':<10} {'Bcast':<10}")
        print("-"*70)
        
        for rate_hz in [5, 10, 20, 30]:
            data = results[rate_hz]
            lost = data['sent'] - data['echoed']
            print(f"{rate_hz:>3} Hz     {data['sent']:<10} {data['echoed']:<10} "
                  f"{lost:<10} {data['success']:.1f}%      {data['broadcasts']:<10}")
        
        print("="*70)
        
        max_reliable = 0
        for rate_hz in [5, 10, 20, 30]:
            if results[rate_hz]['success'] >= 99.0:
                max_reliable = rate_hz
        
        print(f"\nMaximum reliable rate: {max_reliable} Hz (≥99% success)")
        
        total_broadcasts = sum(results[r]['broadcasts'] for r in [5, 10, 20, 30])
        
        if total_broadcasts < 10:
            print("\n⚠ WARNING: Very few broadcasts received during testing")
            print("  Run with --diagnose to troubleshoot UDP communication")
        elif max_reliable >= 20:
            print("✓ Your system can handle F9P GPS rates (20 Hz)!")
        else:
            print("⚠ System struggles above 10 Hz - consider optimization")
        
        print("="*70)
    
    def cleanup(self):
        self.cmd_sock.close()
        self.status_sock.close()

def main():
    parser = argparse.ArgumentParser(
        description='Test cmd_vel at various rates with improved reliability',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 test_cmd_vel_20hz.py                    # 30-second baseline test
  python3 test_cmd_vel_20hz.py --duration 60      # 60-second test
  python3 test_cmd_vel_20hz.py --stress           # Progressive stress test
  python3 test_cmd_vel_20hz.py --diagnose         # Troubleshoot UDP broadcasts
        """
    )
    
    parser.add_argument('--duration', type=int, default=30,
                       help='Test duration in seconds (default: 30)')
    parser.add_argument('--stress', action='store_true',
                       help='Progressive stress test (5-30 Hz)')
    parser.add_argument('--diagnose', action='store_true',
                       help='Diagnostic mode - verify UDP broadcasts')
    
    args = parser.parse_args()
    
    try:
        tester = HighRateTester()
        
        if args.diagnose:
            tester.diagnose_connection(duration=args.duration)
        elif args.stress:
            tester.progressive_stress_test()
        else:
            tester.baseline_20hz_test(duration=args.duration)
        
        tester.cleanup()
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()