#!/usr/bin/env python3
"""
led_status_controller.py - LED Status Controller for Tractor Robot
Monitors UDP navigation data broadcast from rtcm_server.py and displays status on LED tower.
"""

import json
import socket
import threading
import time
from datetime import datetime

import board
import busio
from adafruit_pca9685 import PCA9685

# ==================== Configuration ====================
# LED Channel assignments
CH_RED_R = 8
CH_YELLOW_R = 12
CH_YELLOW_G = 11
CH_GREEN = 9
CH_BLUE = 10

# Inversion flags (set True if LED is active-low)
INVERT_RED = False
INVERT_YELLOW_R = False
INVERT_YELLOW_G = False
INVERT_GREEN = False
INVERT_BLUE = False

# Yellow balance
YELLOW_GAIN_R = 1.00
YELLOW_GAIN_G = 0.90

FREQ_HZ = 500

# UDP settings from rtcm_server.py
UDP_LISTEN_IP = "127.0.0.1"
UDP_LISTEN_PORT = 6002

# Timeout settings
NAV_DATA_TIMEOUT = 2.0  # seconds - consider nav data stale if no updates
BOOT_COMPLETE_TIME = 10.0  # seconds after start

# ==================== PCA9685 Controller ====================
class LEDController:
    """PCA9685 LED controller"""
    
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = FREQ_HZ
        
    def pct_to_16bit(self, pct, invert=False):
        pct = max(0, min(100, int(pct)))
        if invert:
            pct = 100 - pct
        return int(round((pct * 65535) / 100))
    
    def set_led(self, channel, brightness, invert=False):
        """Set single LED channel brightness (0-100)"""
        self.pca.channels[channel].duty_cycle = self.pct_to_16bit(brightness, invert)
    
    def set_yellow(self, brightness):
        """Set yellow LED with proper color balance"""
        brightness = max(0, min(100, int(brightness)))
        r_bright = min(100, int(brightness * YELLOW_GAIN_R))
        g_bright = min(100, int(brightness * YELLOW_GAIN_G))
        self.set_led(CH_YELLOW_R, r_bright, INVERT_YELLOW_R)
        self.set_led(CH_YELLOW_G, g_bright, INVERT_YELLOW_G)
    
    def all_off(self):
        """Turn all LEDs off"""
        self.set_led(CH_RED_R, 0, INVERT_RED)
        self.set_yellow(0)
        self.set_led(CH_GREEN, 0, INVERT_GREEN)
        self.set_led(CH_BLUE, 0, INVERT_BLUE)
    
    def cleanup(self):
        self.all_off()
        self.pca.deinit()

# ==================== Pattern Generator ====================
class PatternGenerator:
    """Generate different LED patterns"""
    
    def __init__(self):
        self.start_time = time.time()
    
    def get_brightness(self, pattern_type, frequency=1.0, max_brightness=100):
        """
        Calculate brightness based on pattern type.
        
        pattern_type: 'off', 'solid', 'blink', 'fade', 'alternating'
        frequency: Hz for blinking/fading patterns
        max_brightness: 0-100
        """
        if pattern_type == "off":
            return 0
        
        if pattern_type == "solid":
            return max_brightness
        
        current_time = time.time() - self.start_time
        period = 1.0 / frequency if frequency > 0 else 1.0
        phase = (current_time % period) / period
        
        if pattern_type == "blink":
            return max_brightness if phase < 0.5 else 0
        
        elif pattern_type == "fade":
            import math
            return int((math.sin(phase * 2 * math.pi - math.pi/2) + 1) * max_brightness / 2)
        
        elif pattern_type == "alternating":
            return max_brightness if phase < 0.5 else int(max_brightness * 0.3)
        
        return 0

# ==================== Navigation State Monitor ====================
class NavStateMonitor:
    """Monitor navigation state from rtcm_server.py UDP broadcasts"""
    
    def __init__(self):
        # Navigation state from UDP
        self.lat = None
        self.lon = None
        self.fix_quality = "Unknown"
        self.heading_deg = None
        self.head_valid = False
        self.carrier = None
        self.expected_err_deg = None
        self.timestamp = None
        
        # Status tracking
        self.last_update_time = 0
        self.boot_time = time.time()
        self.boot_complete = False
        self.nav_data_valid = False
        
        # UDP socket
        self.sock = None
        self.running = False
        
    def start_udp_listener(self):
        """Start listening for UDP navigation data"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((UDP_LISTEN_IP, UDP_LISTEN_PORT))
            self.sock.settimeout(1.0)  # 1 second timeout for recv
            self.running = True
            print(f"✓ UDP listener started on {UDP_LISTEN_IP}:{UDP_LISTEN_PORT}")
            return True
        except OSError as e:
            print(f"✗ Error starting UDP listener: {e}")
            print(f"  Check if port {UDP_LISTEN_PORT} is already in use:")
            print(f"  sudo ss -ulpn | grep {UDP_LISTEN_PORT}")
            return False
        
    def stop_udp_listener(self):
        """Stop UDP listener"""
        self.running = False
        if self.sock:
            self.sock.close()
    
    def udp_listener_thread(self):
        """Background thread to receive UDP navigation data"""
        while self.running:
            try:
                data, _ = self.sock.recvfrom(1024)
                nav_data = json.loads(data.decode('utf-8'))
                
                # Update state
                self.lat = nav_data.get("lat")
                self.lon = nav_data.get("lon")
                self.fix_quality = nav_data.get("fix_quality", "Unknown")
                self.heading_deg = nav_data.get("heading_deg")
                self.head_valid = nav_data.get("headValid", False)
                self.carrier = nav_data.get("carrier")
                self.expected_err_deg = nav_data.get("expectedErrDeg")
                self.timestamp = nav_data.get("timestamp")
                
                self.last_update_time = time.time()
                self.nav_data_valid = True
                
            except socket.timeout:
                # No data received in timeout period - continue
                continue
            except json.JSONDecodeError as e:
                print(f"Error decoding JSON: {e}")
            except Exception as e:
                print(f"UDP listener error: {e}")
                time.sleep(0.1)
    
    def check_timeouts(self):
        """Check for data timeouts and update boot status"""
        current_time = time.time()
        
        # Check nav data timeout
        if current_time - self.last_update_time > NAV_DATA_TIMEOUT:
            self.nav_data_valid = False
            self.fix_quality = "Unknown"
        
        # Check boot completion
        if not self.boot_complete and current_time - self.boot_time > BOOT_COMPLETE_TIME:
            self.boot_complete = True
    
    def has_gps_fix(self):
        """Return True if we have at least a basic GPS fix"""
        return self.nav_data_valid and self.fix_quality not in ["Unknown", "Invalid"]
    
    def has_rtk_fix(self):
        """Return True if we have RTK fixed solution"""
        return self.nav_data_valid and self.fix_quality == "RTK Fixed"
    
    def has_rtk_float(self):
        """Return True if we have RTK float solution"""
        return self.nav_data_valid and self.fix_quality == "RTK Float"
    
    def has_dgps(self):
        """Return True if we have DGPS corrections"""
        return self.nav_data_valid and self.fix_quality == "DGPS"

# ==================== Main LED Status Controller ====================
class LEDStatusController:
    """Main controller coordinating LEDs with navigation state"""
    
    def __init__(self):
        self.led_controller = LEDController()
        self.pattern_gen = PatternGenerator()
        self.nav_monitor = NavStateMonitor()
        self.running = False
        
        # Current LED patterns
        self.yellow_pattern = {"type": "fade", "freq": 0.5, "brightness": 80}
        self.blue_pattern = {"type": "off", "freq": 1.0, "brightness": 100}
        self.green_pattern = {"type": "blink", "freq": 0.5, "brightness": 100}
        self.red_pattern = {"type": "blink", "freq": 1.0, "brightness": 50}
        
    def update_yellow_led(self):
        """
        Yellow LED: GPS/RTK Status
        - Fade: No GPS data
        - 3Hz blink: GPS fix (no RTK)
        - Solid: DGPS or RTK Float
        - 5Hz blink: RTK Fixed (best accuracy)
        """
        if not self.nav_monitor.nav_data_valid:
            # No nav data - boot fade
            self.yellow_pattern = {"type": "fade", "freq": 0.5, "brightness": 80}
        elif self.nav_monitor.has_rtk_fix():
            # RTK Fixed - fast blink (best case!)
            self.yellow_pattern = {"type": "blink", "freq": 5.0, "brightness": 100}
        elif self.nav_monitor.has_rtk_float() or self.nav_monitor.has_dgps():
            # RTK Float or DGPS - solid
            self.yellow_pattern = {"type": "solid", "freq": 0, "brightness": 100}
        elif self.nav_monitor.has_gps_fix():
            # Basic GPS fix - 3Hz blink
            self.yellow_pattern = {"type": "blink", "freq": 3.0, "brightness": 100}
        else:
            # Data is valid but no fix - fade
            self.yellow_pattern = {"type": "fade", "freq": 0.5, "brightness": 80}
    
    def update_green_led(self):
        """
        Green LED: System Health
        - Blink: Booting
        - Solid: System ready
        """
        if self.nav_monitor.boot_complete:
            self.green_pattern = {"type": "solid", "freq": 0, "brightness": 100}
        else:
            self.green_pattern = {"type": "blink", "freq": 0.5, "brightness": 100}
    
    def update_blue_led(self):
        """
        Blue LED: Heading Status
        - OFF: No heading data
        - 1Hz blink: Heading data available but not valid
        - Solid: Valid heading with good accuracy
        - 2Hz blink: Valid heading with poor accuracy
        """
        if not self.nav_monitor.nav_data_valid or self.nav_monitor.heading_deg is None:
            # No heading data
            self.blue_pattern = {"type": "off", "freq": 0, "brightness": 0}
        elif not self.nav_monitor.head_valid:
            # Heading present but not valid
            self.blue_pattern = {"type": "blink", "freq": 1.0, "brightness": 100}
        elif self.nav_monitor.expected_err_deg is not None and self.nav_monitor.expected_err_deg > 5.0:
            # Poor heading accuracy
            self.blue_pattern = {"type": "blink", "freq": 2.0, "brightness": 100}
        else:
            # Good heading - solid
            self.blue_pattern = {"type": "solid", "freq": 0, "brightness": 100}
    
    def update_red_led(self):
        """
        Red LED: Safety Status
        - 1Hz slow blink: System armed/operational (normal)
        - 5Hz fast blink: Nav data timeout (warning)
        - Solid: Critical error (to be implemented)
        """
        if not self.nav_monitor.nav_data_valid and self.nav_monitor.boot_complete:
            # Nav data timeout after boot - warning
            self.red_pattern = {"type": "blink", "freq": 5.0, "brightness": 100}
        else:
            # Normal operation - slow blink
            self.red_pattern = {"type": "blink", "freq": 1.0, "brightness": 50}
    
    def update_led_patterns(self):
        """Update all LED pattern states"""
        self.update_yellow_led()
        self.update_green_led()
        self.update_blue_led()
        self.update_red_led()
    
    def update_led_outputs(self):
        """Update physical LED outputs based on current patterns"""
        yellow_bright = self.pattern_gen.get_brightness(
            self.yellow_pattern["type"],
            self.yellow_pattern["freq"],
            self.yellow_pattern["brightness"]
        )
        
        green_bright = self.pattern_gen.get_brightness(
            self.green_pattern["type"],
            self.green_pattern["freq"],
            self.green_pattern["brightness"]
        )
        
        blue_bright = self.pattern_gen.get_brightness(
            self.blue_pattern["type"],
            self.blue_pattern["freq"],
            self.blue_pattern["brightness"]
        )
        
        red_bright = self.pattern_gen.get_brightness(
            self.red_pattern["type"],
            self.red_pattern["freq"],
            self.red_pattern["brightness"]
        )
        
        # Set physical LEDs
        self.led_controller.set_yellow(yellow_bright)
        self.led_controller.set_led(CH_GREEN, green_bright, INVERT_GREEN)
        self.led_controller.set_led(CH_BLUE, blue_bright, INVERT_BLUE)
        self.led_controller.set_led(CH_RED_R, red_bright, INVERT_RED)
    
    def status_update_thread(self):
        """Background thread to update patterns based on nav state (1Hz)"""
        while self.running:
            self.nav_monitor.check_timeouts()
            self.update_led_patterns()
            time.sleep(1.0)
    
    def led_update_thread(self):
        """Background thread to update LED outputs (20Hz)"""
        while self.running:
            self.update_led_outputs()
            time.sleep(0.05)  # 20Hz
    
    def print_status(self):
        """Print current status to console"""
        nav = self.nav_monitor
        
        # Format lat/lon with None handling
        lat_str = f"{nav.lat:.6f}" if nav.lat is not None else "N/A"
        lon_str = f"{nav.lon:.6f}" if nav.lon is not None else "N/A"
        
        # Format heading with None handling
        if nav.heading_deg is not None:
            heading_str = f"{nav.heading_deg:.1f}°"
        else:
            heading_str = "N/A"
        
        # Format expected error with None handling
        if nav.expected_err_deg is not None:
            err_str = f"{nav.expected_err_deg:.2f}°"
        else:
            err_str = "N/A"
        
        # Format carrier with None handling
        carrier_str = nav.carrier if nav.carrier is not None else "N/A"
        
        # Calculate time since last update
        if nav.last_update_time > 0:
            time_since = time.time() - nav.last_update_time
            update_str = f"{time_since:.1f}s ago"
        else:
            update_str = "Never"
        
        status_str = f"""
=== LED Status Controller ===
Time: {datetime.now().strftime('%H:%M:%S')}
Boot Complete: {nav.boot_complete}
Nav Data Valid: {nav.nav_data_valid}

Navigation State:
  Lat/Lon: {lat_str}, {lon_str}
  Fix Quality: {nav.fix_quality}
  Heading: {heading_str} (valid={nav.head_valid}, err={err_str})
  Carrier: {carrier_str}
  Last Update: {update_str}

LED Patterns:
  Yellow: {self.yellow_pattern['type']:12s} @ {self.yellow_pattern['freq']:.1f} Hz
  Green:  {self.green_pattern['type']:12s} @ {self.green_pattern['freq']:.1f} Hz
  Blue:   {self.blue_pattern['type']:12s} @ {self.blue_pattern['freq']:.1f} Hz
  Red:    {self.red_pattern['type']:12s} @ {self.red_pattern['freq']:.1f} Hz
============================
"""
        print(status_str)
    
    def run(self):
        """Start the LED status controller"""
        print("=" * 60)
        print("LED Status Controller for Tractor Robot")
        print("=" * 60)
        print(f"Listening for navigation data on UDP {UDP_LISTEN_IP}:{UDP_LISTEN_PORT}")
        print("Make sure rtcm_server.py is running!")
        print("=" * 60)
        print()
        
        # Start UDP listener
        if not self.nav_monitor.start_udp_listener():
            print("\n✗ Failed to start UDP listener. Exiting.")
            self.led_controller.cleanup()
            return
        
        self.running = True
        
        # Start background threads
        udp_thread = threading.Thread(
            target=self.nav_monitor.udp_listener_thread, 
            daemon=True
        )
        status_thread = threading.Thread(
            target=self.status_update_thread, 
            daemon=True
        )
        led_thread = threading.Thread(
            target=self.led_update_thread, 
            daemon=True
        )
        
        print("✓ Starting background threads...")
        udp_thread.start()
        status_thread.start()
        led_thread.start()
        
        print("✓ LED controller running")
        print("  Yellow LED should be fading (waiting for GPS data)")
        print("  Press Ctrl+C to exit")
        print()
        
        try:
            # Main loop - print status every 5 seconds
            while True:
                self.print_status()
                time.sleep(5)
        
        except KeyboardInterrupt:
            print("\n" + "=" * 60)
            print("Shutting down gracefully...")
            print("=" * 60)
        
        finally:
            self.running = False
            self.nav_monitor.stop_udp_listener()
            time.sleep(0.2)  # Let threads finish
            self.led_controller.cleanup()
            print("✓ LED Status Controller stopped.")
            print("=" * 60)

# ==================== Main ====================
def main():
    controller = LEDStatusController()
    controller.run()

if __name__ == '__main__':
    main()
