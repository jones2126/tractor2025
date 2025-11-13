#!/usr/bin/env python3
"""
NMEA Frequency Monitor for F9P GPS Units
Monitors the frequency of incoming NMEA sentences from GPS units
and reports statistics including message rate, jitter, and health status.
"""

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nmea_msgs.msg import Sentence
from std_msgs.msg import String, Float32
import collections
import time
import numpy as np


class NMEAFrequencyMonitor:
    def __init__(self):
        rospy.init_node('nmea_frequency_monitor')
        
        # Parameters
        self.expected_rate = rospy.get_param('~expected_rate', 20.0)  # Expected Hz
        self.window_size = rospy.get_param('~window_size', 100)  # Number of messages to average
        self.report_interval = rospy.get_param('~report_interval', 1.0)  # Seconds
        
        # Data storage
        self.message_times = collections.deque(maxlen=self.window_size)
        self.fix_times = collections.deque(maxlen=self.window_size)
        self.nmea_times = collections.deque(maxlen=self.window_size)
        self.inter_message_intervals = collections.deque(maxlen=self.window_size-1)
        
        # Statistics
        self.total_messages = 0
        self.total_fix_messages = 0
        self.total_nmea_messages = 0
        self.start_time = rospy.Time.now()
        self.last_report_time = rospy.Time.now()
        self.last_message_time = None
        self.last_fix_time = None
        self.last_nmea_time = None
        
        # Subscribe to both processed and raw NMEA topics
        self.fix_sub = rospy.Subscriber('/fix', NavSatFix, self.fix_callback, queue_size=10)
        self.nmea_sub = rospy.Subscriber('/nmea_sentence', Sentence, self.nmea_callback, queue_size=50)
        
        # Publishers for monitoring data
        self.rate_pub = rospy.Publisher('/gps_rate_hz', Float32, queue_size=1)
        self.status_pub = rospy.Publisher('/gps_frequency_status', String, queue_size=1)
        
        # Timer for periodic reporting
        self.report_timer = rospy.Timer(rospy.Duration(self.report_interval), self.report_statistics)
        
        rospy.loginfo(f"NMEA Frequency Monitor started. Expected rate: {self.expected_rate} Hz")

    def fix_callback(self, msg):
        """Process NavSatFix messages"""
        current_time = rospy.Time.now()
        self.total_fix_messages += 1
        
        if self.last_fix_time is not None:
            interval = (current_time - self.last_fix_time).to_sec()
            self.inter_message_intervals.append(interval)
        
        self.fix_times.append(current_time.to_sec())
        self.last_fix_time = current_time

    def nmea_callback(self, msg):
        """Process raw NMEA sentences"""
        current_time = rospy.Time.now()
        self.total_nmea_messages += 1
        
        # Track timing for all NMEA sentences
        self.nmea_times.append(current_time.to_sec())
        self.last_nmea_time = current_time
        
        # Log specific sentence types periodically
        if self.total_nmea_messages % 100 == 0:
            sentence_type = msg.sentence.split(',')[0] if msg.sentence else "UNKNOWN"
            rospy.logdebug(f"NMEA sentence type: {sentence_type}")

    def calculate_frequency(self, time_deque):
        """Calculate frequency from a deque of timestamps"""
        if len(time_deque) < 2:
            return 0.0
        
        # Calculate frequency based on time span and message count
        time_span = time_deque[-1] - time_deque[0]
        if time_span > 0:
            return (len(time_deque) - 1) / time_span
        return 0.0

    def calculate_jitter(self):
        """Calculate timing jitter (standard deviation of intervals)"""
        if len(self.inter_message_intervals) < 2:
            return 0.0
        return np.std(self.inter_message_intervals)

    def report_statistics(self, event):
        """Report frequency statistics periodically"""
        current_time = rospy.Time.now()
        
        # Calculate frequencies
        fix_freq = self.calculate_frequency(self.fix_times)
        nmea_freq = self.calculate_frequency(self.nmea_times)
        
        # Calculate overall rates since start
        elapsed_total = (current_time - self.start_time).to_sec()
        overall_fix_rate = self.total_fix_messages / elapsed_total if elapsed_total > 0 else 0
        overall_nmea_rate = self.total_nmea_messages / elapsed_total if elapsed_total > 0 else 0
        
        # Calculate jitter
        jitter_ms = self.calculate_jitter() * 1000  # Convert to milliseconds
        
        # Determine health status
        freq_error = abs(fix_freq - self.expected_rate)
        if freq_error < 0.5:
            health = "EXCELLENT"
        elif freq_error < 2.0:
            health = "GOOD"
        elif freq_error < 5.0:
            health = "DEGRADED"
        else:
            health = "POOR"
        
        # Check for stale data
        fix_age = (current_time - self.last_fix_time).to_sec() if self.last_fix_time else float('inf')
        if fix_age > 2.0:
            health = "NO_DATA"
        
        # Create status message
        status_msg = String()
        status_msg.data = (
            f"GPS Frequency Monitor:\n"
            f"  Fix Rate: {fix_freq:.1f} Hz (recent) / {overall_fix_rate:.1f} Hz (overall)\n"
            f"  NMEA Rate: {nmea_freq:.1f} Hz (recent) / {overall_nmea_rate:.1f} Hz (overall)\n"
            f"  Expected: {self.expected_rate:.1f} Hz\n"
            f"  Jitter: {jitter_ms:.1f} ms\n"
            f"  Total Messages: {self.total_fix_messages} fix / {self.total_nmea_messages} NMEA\n"
            f"  Health: {health}"
        )
        
        # Publish rate
        rate_msg = Float32()
        rate_msg.data = fix_freq
        self.rate_pub.publish(rate_msg)
        self.status_pub.publish(status_msg)
        
        # Log to console
        rospy.loginfo(f"GPS Fix Rate: {fix_freq:.1f} Hz (expected: {self.expected_rate:.1f} Hz) "
                     f"- Jitter: {jitter_ms:.1f} ms - Status: {health}")
        
        # Warn if rate is significantly off
        if freq_error > 5.0 and fix_age < 2.0:
            rospy.logwarn(f"GPS rate deviation: {fix_freq:.1f} Hz (expected {self.expected_rate:.1f} Hz)")
        
        # Log detailed NMEA stats every 10 reports
        if self.total_fix_messages % (10 * self.window_size) == 0:
            self.log_detailed_statistics()

    def log_detailed_statistics(self):
        """Log detailed statistics for debugging"""
        if len(self.inter_message_intervals) > 0:
            intervals_ms = [i * 1000 for i in self.inter_message_intervals]
            rospy.loginfo(
                f"Detailed GPS Statistics:\n"
                f"  Interval Mean: {np.mean(intervals_ms):.1f} ms\n"
                f"  Interval Std: {np.std(intervals_ms):.1f} ms\n"
                f"  Interval Min: {np.min(intervals_ms):.1f} ms\n"
                f"  Interval Max: {np.max(intervals_ms):.1f} ms\n"
                f"  Expected Interval: {1000.0/self.expected_rate:.1f} ms"
            )


def main():
    try:
        monitor = NMEAFrequencyMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
