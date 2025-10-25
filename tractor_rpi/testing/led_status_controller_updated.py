#!/usr/bin/env python3
"""
led_status_controller.py (UPDATED for actual hardware)
========================================================
Controls LED light tower via I2C (PCA9685) based on system status.

HARDWARE CHANGES:
- Updated channel mappings to match actual wiring:
  * RED: Channel 8
  * GREEN: Channel 9
  * BLUE: Channel 10
  * YELLOW: Channels 12 (Red) + 11 (Green) @ 100%/90% ratio
- PWM frequency: 500 Hz (better for these LEDs)
- Added inversion flag support for dim-on-low hardware

Input Data:
1. Port 6003: Teensy and NRF24 Status (from teensy_serial_bridge.py)
2. Port 6002: GPS/RTK state (from rtcm_server.py)

Data Published:
1. I2C statements to PCA9685 to control LED light tower

LED Status Meanings:
- RED: Critical error (no radio signal, no GPS, system fault)
- YELLOW: Warning (degraded GPS, low signal quality)
- GREEN: All systems good (RTK fix, good radio signal)
- BLUE: Auto mode active
"""

import socket
import json
import time
import logging
from collections import defaultdict
import sys

# Try to import I2C libraries
try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    I2C_AVAILABLE = True
except ImportError:
    I2C_AVAILABLE = False
    logging.warning("I2C libraries not available - running in simulation mode")

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('LEDController')

# Configuration
UDP_PORT_TEENSY = 6003  # Teensy/NRF24 status
UDP_PORT_GPS = 6002     # GPS/RTK status
PCA9685_ADDRESS = 0x40  # Default I2C address for PCA9685

# LED channel assignments (UPDATED to match actual hardware)
# Line 55-60: Changed from simple 0-3 to actual hardware channels
CH_RED   = 8    # Standalone Red LED
CH_GREEN = 9    # Standalone Green LED
CH_BLUE  = 10   # Standalone Blue LED
CH_YR    = 12   # Yellow's Red component (100%)
CH_YG    = 11   # Yellow's Green component (90%)

# Line 62-66: Added inversion flags (set True if your LED dims when PWM is low)
INVERT_RED   = False
INVERT_GREEN = False
INVERT_BLUE  = False
INVERT_YR    = False
INVERT_YG    = False

# Line 68-69: Added Yellow LED balance
YELLOW_GAIN_R = 1.00  # 100% red for yellow
YELLOW_GAIN_G = 0.90  # 90% green for yellow

# Line 71-74: Changed to percentage-based brightness levels
LED_OFF = 0      # 0%
LED_DIM = 25     # 25%
LED_MEDIUM = 50  # 50%
LED_BRIGHT = 100 # 100%

# Status timeouts (seconds)
TEENSY_TIMEOUT = 2.0
GPS_TIMEOUT = 5.0

# Line 82-89: New helper functions for percentage to 16-bit PWM conversion
def pct_to_16bit(pct: int, invert: bool) -> int:
    """Convert percentage (0-100) to 16-bit PWM value (0-65535)"""
    pct = max(0, min(100, int(pct)))
    if invert:
        pct = 100 - pct
    return int(round((pct * 65535) / 100))

def set_channel_percent(pca: PCA9685, ch: int, pct: int, invert: bool):
    """Set a PCA9685 channel to a percentage brightness"""
    pca.channels[ch].duty_cycle = pct_to_16bit(pct, invert)


class LEDStatusController:
    def __init__(self):
        """Initialize the LED status controller"""
        self.pca = None
        self.i2c_available = I2C_AVAILABLE
        
        # Latest status data
        self.teensy_status = None
        self.gps_status = None
        self.last_teensy_update = 0
        self.last_gps_update = 0
        
        # Line 115-121: Updated current LED states to track percentages
        self.current_leds = {
            'red': LED_OFF,
            'green': LED_OFF,
            'blue': LED_OFF,
            'yellow': LED_OFF  # Yellow now represents both channels
        }
        
        # Statistics
        self.stats = {
            'teensy_msgs': 0,
            'gps_msgs': 0,
            'led_updates': 0,
            'errors': 0
        }
        
        self.setup()
    
    def setup(self):
        """Setup I2C and UDP connections"""
        # Setup I2C for PCA9685
        if self.i2c_available:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self.pca = PCA9685(i2c, address=PCA9685_ADDRESS)
                # Line 143: Changed frequency from 1000 to 500 Hz
                self.pca.frequency = 500  # 500 Hz works better for these LEDs
                logger.info(f"PCA9685 initialized at address 0x{PCA9685_ADDRESS:02X}")
                logger.info(f"PWM frequency: {self.pca.frequency} Hz")
                
                # Test all LEDs on startup
                self.test_leds()
                
            except Exception as e:
                logger.error(f"Failed to initialize PCA9685: {e}")
                self.i2c_available = False
        
        if not self.i2c_available:
            logger.warning("Running in SIMULATION mode - no actual LED control")
        
        # Setup UDP sockets
        self.teensy_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.teensy_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.teensy_sock.bind(('', UDP_PORT_TEENSY))
        self.teensy_sock.setblocking(False)
        logger.info(f"Listening for Teensy status on UDP port {UDP_PORT_TEENSY}")
        
        self.gps_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gps_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.gps_sock.bind(('', UDP_PORT_GPS))
        self.gps_sock.setblocking(False)
        logger.info(f"Listening for GPS status on UDP port {UDP_PORT_GPS}")
    
    # Line 174-186: New function to turn off all LEDs
    def all_off(self):
        """Turn off all LEDs"""
        if self.i2c_available and self.pca:
            set_channel_percent(self.pca, CH_RED,   0, INVERT_RED)
            set_channel_percent(self.pca, CH_GREEN, 0, INVERT_GREEN)
            set_channel_percent(self.pca, CH_BLUE,  0, INVERT_BLUE)
            set_channel_percent(self.pca, CH_YR,    0, INVERT_YR)
            set_channel_percent(self.pca, CH_YG,    0, INVERT_YG)
        
        self.current_leds = {
            'red': LED_OFF,
            'green': LED_OFF,
            'blue': LED_OFF,
            'yellow': LED_OFF
        }
    
    # Line 188-213: Completely rewritten set_led function for new hardware
    def set_led(self, color, brightness_pct):
        """
        Set LED brightness (0-100%)
        
        Args:
            color: 'red', 'green', 'blue', or 'yellow'
            brightness_pct: 0-100 percentage
        """
        brightness_pct = max(0, min(100, int(brightness_pct)))
        
        if self.i2c_available and self.pca:
            try:
                if color == 'red':
                    # Turn on red, ensure yellow components off
                    set_channel_percent(self.pca, CH_RED, brightness_pct, INVERT_RED)
                    set_channel_percent(self.pca, CH_GREEN, 0, INVERT_GREEN)
                    set_channel_percent(self.pca, CH_BLUE, 0, INVERT_BLUE)
                    set_channel_percent(self.pca, CH_YR, 0, INVERT_YR)
                    set_channel_percent(self.pca, CH_YG, 0, INVERT_YG)
                
                elif color == 'green':
                    # Turn on green, ensure yellow components off
                    set_channel_percent(self.pca, CH_RED, 0, INVERT_RED)
                    set_channel_percent(self.pca, CH_GREEN, brightness_pct, INVERT_GREEN)
                    set_channel_percent(self.pca, CH_BLUE, 0, INVERT_BLUE)
                    set_channel_percent(self.pca, CH_YR, 0, INVERT_YR)
                    set_channel_percent(self.pca, CH_YG, 0, INVERT_YG)
                
                elif color == 'blue':
                    # Turn on blue, ensure yellow components off
                    set_channel_percent(self.pca, CH_RED, 0, INVERT_RED)
                    set_channel_percent(self.pca, CH_GREEN, 0, INVERT_GREEN)
                    set_channel_percent(self.pca, CH_BLUE, brightness_pct, INVERT_BLUE)
                    set_channel_percent(self.pca, CH_YR, 0, INVERT_YR)
                    set_channel_percent(self.pca, CH_YG, 0, INVERT_YG)
                
                elif color == 'yellow':
                    # Yellow uses TWO channels with specific ratios
                    r_pct = int(round(brightness_pct * YELLOW_GAIN_R))
                    g_pct = int(round(brightness_pct * YELLOW_GAIN_G))
                    r_pct = min(r_pct, 100)
                    g_pct = min(g_pct, 100)
                    
                    # Turn on yellow components, ensure standalone RGB off
                    set_channel_percent(self.pca, CH_RED, 0, INVERT_RED)
                    set_channel_percent(self.pca, CH_GREEN, 0, INVERT_GREEN)
                    set_channel_percent(self.pca, CH_BLUE, 0, INVERT_BLUE)
                    set_channel_percent(self.pca, CH_YR, r_pct, INVERT_YR)
                    set_channel_percent(self.pca, CH_YG, g_pct, INVERT_YG)
                
                else:
                    logger.error(f"Unknown LED color: {color}")
                    return
                    
            except Exception as e:
                logger.error(f"Error setting LED {color}: {e}")
                self.stats['errors'] += 1
        
        # Update current state
        if self.current_leds[color] != brightness_pct:
            self.current_leds[color] = brightness_pct
            self.stats['led_updates'] += 1
            
            # Log LED changes
            if brightness_pct > 0:
                logger.info(f"LED {color.upper()}: {brightness_pct}%")
            else:
                logger.info(f"LED {color.upper()}: OFF")
    
    def test_leds(self):
        """Test all LEDs on startup"""
        logger.info("Testing LEDs...")
        
        colors = ['red', 'green', 'blue', 'yellow']
        for color in colors:
            logger.info(f"Testing {color.upper()}...")
            self.set_led(color, LED_BRIGHT)
            time.sleep(0.5)
            self.set_led(color, LED_OFF)
            time.sleep(0.2)
        
        logger.info("LED test complete")
    
    def receive_teensy_status(self):
        """Receive and parse Teensy status from UDP"""
        try:
            data, addr = self.teensy_sock.recvfrom(4096)
            message = json.loads(data.decode())
            
            self.teensy_status = message
            self.last_teensy_update = time.time()
            self.stats['teensy_msgs'] += 1
            
            return True
        except socket.error:
            pass  # No data available
        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse Teensy JSON: {e}")
            self.stats['errors'] += 1
        
        return False
    
    def receive_gps_status(self):
        """Receive and parse GPS status from UDP"""
        try:
            data, addr = self.gps_sock.recvfrom(4096)
            message = json.loads(data.decode())
            
            self.gps_status = message
            self.last_gps_update = time.time()
            self.stats['gps_msgs'] += 1
            
            return True
        except socket.error:
            pass  # No data available
        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse GPS JSON: {e}")
            self.stats['errors'] += 1
        
        return False
    
    def determine_led_status(self):
        """
        Determine LED states based on system status
        
        Priority (highest to lowest):
        1. RED: Critical errors
        2. YELLOW: Warnings
        3. BLUE: Auto mode
        4. GREEN: All good
        """
        current_time = time.time()
        
        # Default all off
        leds = {
            'red': LED_OFF,
            'yellow': LED_OFF,
            'green': LED_OFF,
            'blue': LED_OFF
        }
        
        # Check for critical errors (RED)
        # 1. No Teensy data (timeout)
        if current_time - self.last_teensy_update > TEENSY_TIMEOUT:
            leds['red'] = LED_BRIGHT
            return leds
        
        # 2. No radio signal
        if self.teensy_status:
            radio_signal = self.teensy_status.get('radio', {}).get('signal', 'UNKNOWN')
            if radio_signal != 'GOOD':
                leds['red'] = LED_BRIGHT
                return leds
        
        # 3. No GPS data (timeout)
        if current_time - self.last_gps_update > GPS_TIMEOUT:
            leds['red'] = LED_BRIGHT
            return leds
        
        # Check for warnings (YELLOW)
        # 1. GPS not RTK fixed
        if self.gps_status:
            gps_fix = self.gps_status.get('fix_type', 0)
            if gps_fix not in [4, 5]:  # Not RTK Fixed or Float
                leds['yellow'] = LED_BRIGHT
        
        # 2. Low radio signal quality (if we add SNR/RSSI later)
        if self.teensy_status:
            radio_rate = self.teensy_status.get('radio', {}).get('ack_rate', 0)
            if radio_rate < 8.0:  # Below 8 Hz is concerning
                leds['yellow'] = LED_MEDIUM
        
        # Check for auto mode (BLUE)
        if self.teensy_status:
            trans_mode = self.teensy_status.get('transmission', {}).get('mode', 0)
            if trans_mode == 2:  # Auto mode
                leds['blue'] = LED_BRIGHT
        
        # All good (GREEN)
        if leds['red'] == LED_OFF and leds['yellow'] == LED_OFF:
            leds['green'] = LED_BRIGHT
        
        return leds
    
    def update_leds(self):
        """Update LED states based on current system status"""
        led_states = self.determine_led_status()
        
        # Update each LED (set_led now handles turning others off)
        for color, brightness in led_states.items():
            if brightness != self.current_leds[color]:
                self.set_led(color, brightness)
    
    def print_statistics(self):
        """Print statistics"""
        logger.info(f"Stats - Teensy msgs: {self.stats['teensy_msgs']}, "
                   f"GPS msgs: {self.stats['gps_msgs']}, "
                   f"LED updates: {self.stats['led_updates']}, "
                   f"Errors: {self.stats['errors']}")
        
        # Print current system status
        if self.teensy_status:
            radio = self.teensy_status.get('radio', {})
            trans = self.teensy_status.get('transmission', {})
            logger.info(f"Teensy - Radio: {radio.get('signal', 'N/A')}, "
                       f"Mode: {trans.get('mode', 'N/A')}")
        
        if self.gps_status:
            logger.info(f"GPS - Fix: {self.gps_status.get('fix_type', 'N/A')}")
    
    def run(self):
        """Main loop"""
        logger.info("LED Status Controller starting...")
        logger.info(f"LED Channel Mapping: RED={CH_RED}, GREEN={CH_GREEN}, "
                   f"BLUE={CH_BLUE}, YELLOW=({CH_YR}+{CH_YG})")
        logger.info(f"Listening on ports {UDP_PORT_TEENSY} (Teensy) and {UDP_PORT_GPS} (GPS)")
        
        last_stats_print = time.time()
        stats_interval = 30.0  # Print stats every 30 seconds
        
        try:
            while True:
                # Receive status updates
                self.receive_teensy_status()
                self.receive_gps_status()
                
                # Update LEDs based on current status
                self.update_leds()
                
                # Print statistics periodically
                if time.time() - last_stats_print >= stats_interval:
                    self.print_statistics()
                    last_stats_print = time.time()
                
                # Sleep to prevent CPU spinning
                time.sleep(0.1)  # 10 Hz update rate
        
        except KeyboardInterrupt:
            logger.info("Shutting down...")
        finally:
            self.cleanup()
    
    # Line 470-481: Updated cleanup function
    def cleanup(self):
        """Cleanup resources"""
        # Turn off all LEDs using new all_off method
        logger.info("Turning off all LEDs...")
        self.all_off()
        
        # Close sockets
        self.teensy_sock.close()
        self.gps_sock.close()
        
        # Deinitialize PCA9685
        if self.pca:
            self.pca.deinit()
        
        self.print_statistics()
        logger.info("Cleanup complete")


def main():
    """Main entry point"""
    try:
        controller = LEDStatusController()
        controller.run()
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        raise


if __name__ == "__main__":
    main()
