#!/usr/bin/env python3
"""
robot_status_leds.py
LED Status Control System for Robot Signal Tower
Monitors robot subsystems and updates LED status indicators
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import threading
from enum import Enum

import board
import busio
from adafruit_pca9685 import PCA9685

# Import ROS message types (adjust based on your actual messages)
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry

# ==================== LED Channel Assignments ====================
CH_RED_R = 8
CH_YELLOW_R = 12
CH_YELLOW_G = 11
CH_GREEN = 9
CH_BLUE = 10

# Inversion flags (set True if your LEDs are active-low)
INVERT_RED = False
INVERT_YELLOW_R = False
INVERT_YELLOW_G = False
INVERT_GREEN = False
INVERT_BLUE = False

# Yellow LED balance (adjust to your preference)
YELLOW_GAIN_R = 1.00
YELLOW_GAIN_G = 0.90

# PWM frequency
FREQ_HZ = 500

# ==================== LED States Enum ====================
class YellowState(Enum):
    BOOT_FADE = 1          # Slow fade during boot
    GPS_BLINK = 2          # 3Hz blink when GPS active
    RTCM_SOLID = 3         # Solid when RTCM received
    RTK_FAST_BLINK = 4     # 5Hz blink when RTK fixed
    RTK_LOST_WARN = 5      # Alternating when RTK lost

class BlueState(Enum):
    NO_COMM = 1            # OFF - no communication
    NRF_MANUAL = 2         # 1Hz blink - NRF24 manual mode
    WIFI_MANUAL = 3        # 2Hz blink - WiFi manual mode
    AUTO_RTK_READY = 4     # Solid - Auto mode with RTK
    AUTO_RTK_LOST = 5      # 5Hz blink - Auto paused, needs manual
    AUTO_EXECUTING = 6     # Alternating - executing mission

class GreenState(Enum):
    OFF = 1                # System off/initializing
    STARTING = 2           # 0.5Hz blink - starting up
    NOMINAL = 3            # Solid - all systems good
    WARNING = 4            # 2Hz blink - minor warning
    CRITICAL = 5           # 5Hz blink - critical warning

class RedState(Enum):
    OFF = 1                # No errors
    ESTOP_ARMED = 2        # 1Hz blink - e-stop armed
    SAFETY_VIOLATION = 3   # 5Hz blink - safety issue
    ESTOP_ACTIVE = 4       # Solid - E-STOP triggered
    RECOVERABLE_ERROR = 5  # Alternating - recoverable error

# ==================== LED Pattern Class ====================
class LEDPattern:
    """Manages LED brightness patterns over time"""
    def __init__(self):
        self.pattern_type = "off"  # off, solid, blink, fade, alternating
        self.frequency = 0         # Hz for blink/alternating
        self.brightness = 0        # 0-100 for solid
        self.last_update = time.time()
        self.phase = 0             # For tracking blink/fade cycles
        
    def get_brightness(self):
        """Calculate current brightness based on pattern"""
        current_time = time.time()
        
        if self.pattern_type == "off":
            return 0
        elif self.pattern_type == "solid":
            return self.brightness
        elif self.pattern_type == "blink":
            period = 1.0 / self.frequency if self.frequency > 0 else 1.0
            phase = (current_time % period) / period
            return self.brightness if phase < 0.5 else 0
        elif self.pattern_type == "fade":
            # Smooth sine wave fade
            period = 2.0 / self.frequency if self.frequency > 0 else 2.0
            phase = (current_time % period) / period
            import math
            brightness = int((math.sin(phase * 2 * math.pi - math.pi/2) + 1) * self.brightness / 2)
            return brightness
        elif self.pattern_type == "alternating":
            # Quick alternation between two brightness levels
            period = 1.0 / self.frequency if self.frequency > 0 else 1.0
            phase = (current_time % period) / period
            return self.brightness if phase < 0.5 else int(self.brightness * 0.3)
        
        return 0
    
    def set_pattern(self, pattern_type, frequency=0, brightness=100):
        """Set the LED pattern parameters"""
        self.pattern_type = pattern_type
        self.frequency = frequency
        self.brightness = brightness
        self.last_update = time.time()

# ==================== PCA9685 Control Class ====================
class PCA9685Controller:
    """Manages PCA9685 PWM controller"""
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = FREQ_HZ
        
    def pct_to_16bit(self, pct: int, invert: bool) -> int:
        """Convert percentage to 16-bit PWM value"""
        pct = max(0, min(100, int(pct)))
        if invert:
            pct = 100 - pct
        return int(round((pct * 65535) / 100))
    
    def set_channel_percent(self, ch: int, pct: int, invert: bool):
        """Set channel to percentage brightness"""
        self.pca.channels[ch].duty_cycle = self.pct_to_16bit(pct, invert)
    
    def set_yellow(self, pct: int):
        """Set yellow LED with proper R/G balance"""
        pct = max(0, min(100, int(pct)))
        r_pct = min(100, int(round(pct * YELLOW_GAIN_R)))
        g_pct = min(100, int(round(pct * YELLOW_GAIN_G)))
        self.set_channel_percent(CH_YELLOW_R, r_pct, INVERT_YELLOW_R)
        self.set_channel_percent(CH_YELLOW_G, g_pct, INVERT_YELLOW_G)
    
    def all_off(self):
        """Turn all LEDs off"""
        self.set_channel_percent(CH_RED_R, 0, INVERT_RED)
        self.set_channel_percent(CH_YELLOW_R, 0, INVERT_YELLOW_R)
        self.set_channel_percent(CH_YELLOW_G, 0, INVERT_YELLOW_G)
        self.set_channel_percent(CH_GREEN, 0, INVERT_GREEN)
        self.set_channel_percent(CH_BLUE, 0, INVERT_BLUE)
    
    def cleanup(self):
        """Cleanup PCA9685"""
        self.all_off()
        self.pca.deinit()

# ==================== Robot Status LED Node ====================
class RobotStatusLEDs(Node):
    """ROS 2 node to monitor robot status and control LEDs"""
    
    def __init__(self):
        super().__init__('robot_status_leds')
        
        # Initialize PCA9685 controller
        self.led_controller = PCA9685Controller()
        
        # Initialize LED patterns
        self.yellow_pattern = LEDPattern()
        self.blue_pattern = LEDPattern()
        self.green_pattern = LEDPattern()
        self.red_pattern = LEDPattern()
        
        # System status variables
        self.boot_complete = False
        self.gps_active = False
        self.rtcm_active = False
        self.rtk_fixed = False
        self.nrf_connected = False
        self.wifi_connected = False
        self.control_mode = "manual"  # manual, auto
        self.estop_triggered = False
        self.last_gps_time = 0
        self.last_rtcm_time = 0
        self.last_nrf_time = 0
        
        # Create QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ===== ROS 2 Subscribers =====
        # GPS subscriber
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            qos_profile
        )
        
        # RTCM status subscriber (you'll need to create this topic)
        self.rtcm_sub = self.create_subscription(
            Bool,
            '/rtcm_status',
            self.rtcm_callback,
            qos_profile
        )
        
        # Control mode subscriber
        self.control_mode_sub = self.create_subscription(
            String,
            '/control_mode',
            self.control_mode_callback,
            qos_profile
        )
        
        # NRF24 status subscriber
        self.nrf_sub = self.create_subscription(
            Bool,
            '/nrf_status',
            self.nrf_callback,
            qos_profile
        )
        
        # E-stop subscriber
        self.estop_sub = self.create_subscription(
            Bool,
            '/estop',
            self.estop_callback,
            qos_profile
        )
        
        # Create timer for LED updates (20Hz update rate)
        self.led_timer = self.create_timer(0.05, self.update_leds)
        
        # Create timer for status checks (1Hz)
        self.status_timer = self.create_timer(1.0, self.check_timeouts)
        
        # Start with boot sequence
        self.set_boot_sequence()
        
        self.get_logger().info('Robot Status LED Controller initialized')
    
    # ===== Callback Methods =====
    def gps_callback(self, msg):
        """Handle GPS fix messages"""
        self.gps_active = True
        self.last_gps_time = time.time()
        
        # Check RTK status (status values: -1=no fix, 0=GPS, 1=DGPS, 2=RTK)
        if msg.status.status == 2:
            self.rtk_fixed = True
        else:
            self.rtk_fixed = False
        
        self.update_yellow_state()
    
    def rtcm_callback(self, msg):
        """Handle RTCM status messages"""
        self.rtcm_active = msg.data
        self.last_rtcm_time = time.time()
        self.update_yellow_state()
    
    def control_mode_callback(self, msg):
        """Handle control mode changes"""
        self.control_mode = msg.data
        self.update_blue_state()
    
    def nrf_callback(self, msg):
        """Handle NRF24 radio status"""
        self.nrf_connected = msg.data
        self.last_nrf_time = time.time()
        self.update_blue_state()
    
    def estop_callback(self, msg):
        """Handle e-stop status"""
        self.estop_triggered = msg.data
        self.update_red_state()
    
    # ===== Status Update Methods =====
    def check_timeouts(self):
        """Check for communication timeouts"""
        current_time = time.time()
        
        # GPS timeout (3 seconds)
        if current_time - self.last_gps_time > 3.0:
            self.gps_active = False
            self.rtk_fixed = False
            self.update_yellow_state()
        
        # RTCM timeout (5 seconds)
        if current_time - self.last_rtcm_time > 5.0:
            self.rtcm_active = False
            self.update_yellow_state()
        
        # NRF timeout (2 seconds)
        if current_time - self.last_nrf_time > 2.0:
            self.nrf_connected = False
            self.update_blue_state()
        
        # Boot completion check (after 10 seconds)
        if not self.boot_complete and current_time > 10.0:
            self.boot_complete = True
            self.update_green_state()
    
    def set_boot_sequence(self):
        """Set initial boot sequence patterns"""
        # Yellow: slow fade during boot
        self.yellow_pattern.set_pattern("fade", frequency=0.5, brightness=80)
        
        # Blue: off during boot
        self.blue_pattern.set_pattern("off")
        
        # Green: slow blink during boot
        self.green_pattern.set_pattern("blink", frequency=0.5, brightness=100)
        
        # Red: off (no errors)
        self.red_pattern.set_pattern("off")
    
    def update_yellow_state(self):
        """Update yellow LED based on GPS/RTCM/RTK status"""
        if not self.gps_active:
            # Boot fade - no GPS yet
            self.yellow_pattern.set_pattern("fade", frequency=0.5, brightness=80)
        elif self.rtk_fixed:
            # RTK Fix achieved - fast blink
            self.yellow_pattern.set_pattern("blink", frequency=5.0, brightness=100)
        elif self.rtcm_active:
            # RTCM being received - solid
            self.yellow_pattern.set_pattern("solid", brightness=100)
        elif self.gps_active:
            # GPS active, no RTCM - 3Hz blink
            self.yellow_pattern.set_pattern("blink", frequency=3.0, brightness=100)
    
    def update_blue_state(self):
        """Update blue LED based on control mode and communication"""
        if self.control_mode == "auto":
            if self.rtk_fixed:
                # Auto mode with RTK - solid (ready)
                self.blue_pattern.set_pattern("solid", brightness=100)
            else:
                # Auto mode without RTK - fast blink (paused)
                self.blue_pattern.set_pattern("blink", frequency=5.0, brightness=100)
        elif self.nrf_connected:
            # Manual mode with NRF - slow blink
            self.blue_pattern.set_pattern("blink", frequency=1.0, brightness=100)
        elif self.wifi_connected:
            # Manual mode with WiFi - medium blink
            self.blue_pattern.set_pattern("blink", frequency=2.0, brightness=100)
        else:
            # No communication
            self.blue_pattern.set_pattern("off")
    
    def update_green_state(self):
        """Update green LED based on system health"""
        if self.boot_complete:
            # All systems nominal
            self.green_pattern.set_pattern("solid", brightness=100)
        else:
            # Still booting
            self.green_pattern.set_pattern("blink", frequency=0.5, brightness=100)
    
    def update_red_state(self):
        """Update red LED based on safety status"""
        if self.estop_triggered:
            # E-stop active - solid red
            self.red_pattern.set_pattern("solid", brightness=100)
        else:
            # E-stop armed but not triggered - slow blink
            self.red_pattern.set_pattern("blink", frequency=1.0, brightness=50)
    
    def update_leds(self):
        """Update LED outputs based on current patterns (20Hz)"""
        # Get current brightness for each LED
        yellow_brightness = self.yellow_pattern.get_brightness()
        blue_brightness = self.blue_pattern.get_brightness()
        green_brightness = self.green_pattern.get_brightness()
        red_brightness = self.red_pattern.get_brightness()
        
        # Update physical LEDs
        self.led_controller.set_yellow(yellow_brightness)
        self.led_controller.set_channel_percent(CH_BLUE, blue_brightness, INVERT_BLUE)
        self.led_controller.set_channel_percent(CH_GREEN, green_brightness, INVERT_GREEN)
        self.led_controller.set_channel_percent(CH_RED_R, red_brightness, INVERT_RED)
    
    def cleanup(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down LED controller...')
        self.led_controller.cleanup()

# ==================== Main ====================
def main(args=None):
    rclpy.init(args=args)
    
    led_node = RobotStatusLEDs()
    
    try:
        rclpy.spin(led_node)
    except KeyboardInterrupt:
        led_node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        led_node.cleanup()
        led_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
