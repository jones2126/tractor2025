"""
navigation_reader.py
===================
Navigation system placeholder script that reads GPS data from the dual GPS RTK client
via Unix socket and processes it for navigation calculations.

This script demonstrates reading the comma-delimited GPS data and can be expanded
to include path planning, Pure Pursuit, and other navigation algorithms.
"""

import socket
import time
import logging
import threading
from datetime import datetime
import math

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('logs/navigation_reader.log')
    ]
)
logger = logging.getLogger('NavigationReader')

# Configuration
NAV_SOCKET_PATH = "/tmp/gps_nav_data.sock"
RECONNECT_DELAY = 2

class NavigationData:
    """Class to store and manage current navigation data"""
    def __init__(self):
        self.timestamp = None
        self.base_lat = None
        self.base_lon = None
        self.base_altitude = None
        self.rtk_status_code = None
        self.calculated_heading = None
        self.num_satellites = None
        self.last_update = None
        
        # Navigation state
        self.is_rtk_ready = False
        self.heading_valid = False
        self.position_valid = False
        
        # Rate calculation variables
        self.update_count = 0
        self.rate_start_time = time.time()
        self.current_rate_hz = 0.0
        self.rate_history = []  # Store last 10 rate measurements
        self.last_rate_calculation = time.time()
        
    def update_from_csv(self, csv_line):
        """
        Update navigation data from CSV format:
        gps,timestamp,lat,lon,altitude,rtk_status_code,heading,satellites
        """
        try:
            parts = csv_line.strip().split(',')
            if len(parts) < 8 or parts[0] != 'gps':
                return False
                
            self.timestamp = float(parts[1])
            self.base_lat = float(parts[2]) if parts[2] != '0' else None
            self.base_lon = float(parts[3]) if parts[3] != '0' else None  
            self.base_altitude = float(parts[4]) if parts[4] != '0' else None
            self.rtk_status_code = parts[5]
            self.calculated_heading = float(parts[6]) if parts[6] != '0' else None
            self.num_satellites = int(parts[7])
            self.last_update = time.time()
            
            # Update navigation state flags
            self.is_rtk_ready = self.rtk_status_code in ['4', '5']  # RTK Fixed or Float
            self.heading_valid = self.calculated_heading is not None
            self.position_valid = all([self.base_lat, self.base_lon])
            
            # Update rate calculation
            self.update_count += 1
            self._calculate_update_rate()
            
            return True
            
        except Exception as e:
            logger.error(f"Error parsing CSV data: {e}")
            return False
    
    def _calculate_update_rate(self):
        """Calculate the GPS data update rate in Hz."""
        current_time = time.time()
        
        # Calculate rate every 5 seconds
        if current_time - self.last_rate_calculation >= 5.0:
            elapsed_time = current_time - self.rate_start_time
            
            if elapsed_time > 0:
                self.current_rate_hz = self.update_count / elapsed_time
                
                # Store in history (keep last 10 measurements)
                self.rate_history.append(self.current_rate_hz)
                if len(self.rate_history) > 10:
                    self.rate_history.pop(0)
                
                # Reset counters for next measurement period
                self.update_count = 0
                self.rate_start_time = current_time
                self.last_rate_calculation = current_time
    
    def get_average_rate_hz(self):
        """Get the average update rate over the last 10 measurements."""
        if not self.rate_history:
            return 0.0
        return sum(self.rate_history) / len(self.rate_history)
    
    def get_rate_stats(self):
        """Get detailed rate statistics."""
        if not self.rate_history:
            return {
                "current_hz": 0.0,
                "average_hz": 0.0,
                "min_hz": 0.0,
                "max_hz": 0.0,
                "measurements": 0
            }
        
        return {
            "current_hz": self.current_rate_hz,
            "average_hz": self.get_average_rate_hz(),
            "min_hz": min(self.rate_history),
            "max_hz": max(self.rate_history),
            "measurements": len(self.rate_history)
        }
    
    def get_status_summary(self):
        """Get a human-readable status summary"""
        status_map = {
            "0": "No Fix",
            "1": "GPS Fix", 
            "2": "DGPS Fix",
            "4": "RTK Fixed",
            "5": "RTK Float"
        }
        
        rtk_status = status_map.get(self.rtk_status_code, "Unknown")
        
        return {
            "rtk_status": rtk_status,
            "position_valid": self.position_valid,
            "heading_valid": self.heading_valid,
            "satellites": self.num_satellites,
            "lat": self.base_lat,
            "lon": self.base_lon,
            "heading": self.calculated_heading,
            "altitude": self.base_altitude,
            "rate_stats": self.get_rate_stats()
        }

def connect_to_gps_socket():
    """Connect to the GPS data Unix socket with retry logic."""
    while True:
        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.connect(NAV_SOCKET_PATH)
            logger.info(f"Connected to GPS data socket at {NAV_SOCKET_PATH}")
            return sock
        except Exception as e:
            logger.error(f"Failed to connect to GPS socket: {e}. Retrying in {RECONNECT_DELAY} seconds...")
            time.sleep(RECONNECT_DELAY)

def calculate_distance(lat1, lon1, lat2, lon2):
    """
    Calculate distance between two GPS coordinates using Haversine formula.
    Returns distance in meters.
    """
    if None in [lat1, lon1, lat2, lon2]:
        return None
        
    # Convert to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    
    # Earth's radius in meters
    earth_radius = 6371000
    distance = earth_radius * c
    
    return distance

def navigation_control_loop(nav_data):
    """
    Main navigation control loop - placeholder for actual navigation logic.
    This is where you would implement:
    - Path following (Pure Pursuit)
    - Obstacle avoidance
    - Mission planning
    - Speed control
    """
    last_status_time = time.time()
    status_interval = 5  # Report status every 5 seconds
    
    # Navigation state variables
    target_waypoint = None
    mission_active = False
    
    while True:
        current_time = time.time()
        
        # Check if we have valid navigation data
        if nav_data.last_update and (current_time - nav_data.last_update) < 2.0:
            status = nav_data.get_status_summary()
            
            # Report status periodically
            if current_time - last_status_time >= status_interval:
                logger.info("=== Navigation Status Report ===")
                logger.info(f"RTK Status: {status['rtk_status']}")
                logger.info(f"Position Valid: {status['position_valid']}")
                logger.info(f"Heading Valid: {status['heading_valid']}")
                logger.info(f"Satellites: {status['satellites']}")
                
                # Report GPS data rates
                rate_stats = status['rate_stats']
                logger.info(f"📊 GPS Data Rate: {rate_stats['current_hz']:.2f} Hz (current)")
                logger.info(f"📈 Average Rate: {rate_stats['average_hz']:.2f} Hz over {rate_stats['measurements']} measurements")
                if rate_stats['measurements'] > 1:
                    logger.info(f"📉 Rate Range: {rate_stats['min_hz']:.2f} - {rate_stats['max_hz']:.2f} Hz")
                
                # Rate health assessment
                expected_rate = 5.0  # We expect 5Hz from the dual GPS client
                if rate_stats['current_hz'] < expected_rate * 0.8:
                    logger.warning(f"⚠️  GPS data rate is low! Expected ~{expected_rate}Hz, got {rate_stats['current_hz']:.2f}Hz")
                elif rate_stats['current_hz'] > expected_rate * 1.2:
                    logger.warning(f"⚠️  GPS data rate is unusually high! Expected ~{expected_rate}Hz, got {rate_stats['current_hz']:.2f}Hz")
                else:
                    logger.info(f"✅ GPS data rate is healthy ({rate_stats['current_hz']:.2f}Hz)")
                
                if status['position_valid']:
                    logger.info(f"Position: {status['lat']:.8f}, {status['lon']:.8f}")
                    logger.info(f"Altitude: {status['altitude']:.2f}m")
                    
                if status['heading_valid']:
                    logger.info(f"Heading: {status['heading']:.2f}°")
                
                # Navigation readiness check
                nav_ready = (status['rtk_status'] in ['RTK Fixed', 'RTK Float'] and 
                           status['position_valid'] and 
                           status['heading_valid'])
                           
                logger.info(f"Navigation Ready: {nav_ready}")
                
                if nav_ready:
                    logger.info("🚜 TRACTOR READY FOR AUTONOMOUS NAVIGATION! 🚜")
                    
                    # Example navigation logic placeholder
                    if not mission_active:
                        logger.info("📍 Waiting for mission waypoints...")
                        # Here you would:
                        # - Load waypoints from file
                        # - Start path following algorithm
                        # - Set mission_active = True
                        
                    else:
                        logger.info("🎯 Executing navigation mission...")
                        # Here you would:
                        # - Calculate steering commands
                        # - Calculate speed commands  
                        # - Send commands to low-level control
                        # - Check for waypoint completion
                
                logger.info("=" * 35)
                last_status_time = current_time
                
        else:
            if current_time - last_status_time >= status_interval:
                logger.warning("⚠️  No valid GPS data received recently")
                last_status_time = current_time
        
        time.sleep(0.1)  # 10Hz control loop

def main():
    """Main function for navigation reader."""
    logger.info("Starting Navigation Reader...")
    
    # Initialize navigation data storage
    nav_data = NavigationData()
    
    # Start navigation control loop in separate thread
    nav_thread = threading.Thread(target=navigation_control_loop, args=(nav_data,), daemon=True)
    nav_thread.start()
    
    # Main GPS data reading loop
    while True:
        gps_socket = connect_to_gps_socket()
        
        try:
            # Set socket to non-blocking for timeout
            gps_socket.settimeout(1.0)
            
            buffer = ""
            logger.info("📡 Started reading GPS navigation data...")
            
            while True:
                try:
                    data = gps_socket.recv(1024).decode('utf-8')
                    if not data:
                        logger.warning("GPS socket connection closed")
                        break
                        
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        
                        if line.strip():
                            success = nav_data.update_from_csv(line.strip())
                            if success:
                                # Optional: Log every GPS update with rate info (might be verbose)
                                rate_stats = nav_data.get_rate_stats()
                                logger.debug(f"GPS Update: Rate={rate_stats['current_hz']:.1f}Hz | {line.strip()}")
                            else:
                                logger.warning(f"Failed to parse GPS data: {line.strip()}")
                                
                except socket.timeout:
                    # Timeout is normal, just continue
                    continue
                except Exception as e:
                    logger.error(f"Error reading from GPS socket: {e}")
                    break
                    
        except Exception as e:
            logger.error(f"Error in GPS data loop: {e}")
        finally:
            gps_socket.close()
            
        logger.info("Reconnecting to GPS socket...")
        time.sleep(RECONNECT_DELAY)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger.info("Navigation reader shutting down...")
    except Exception as e:
        logger.error(f"Fatal error in navigation reader: {e}")
        raise