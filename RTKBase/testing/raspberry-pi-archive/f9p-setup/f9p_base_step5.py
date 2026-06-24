import serial
import logging
import time
import re

# Configuration
PORT = "/dev/f9p"
BAUDRATE = 115200
MONITOR_DURATION = 30

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)

def extract_gga_from_mixed_data(data_chunk):
    """
    Extract GGA sentences from mixed binary/text data stream.
    This handles the case where RTCM and NMEA are mixed together.
    """
    gga_sentences = []
    
    try:
        # Try to decode as ASCII, replacing any non-ASCII bytes
        text_data = data_chunk.decode('ascii', errors='replace')
        
        # Use regex to find GGA sentences in the mixed data
        # Pattern: $[GP|GN|GL]GGA followed by comma-separated fields ending with *XX
        gga_pattern = r'\$G[NPLS]GGA,[^*]*\*[0-9A-F]{2}'
        matches = re.findall(gga_pattern, text_data)
        
        for match in matches:
            # Clean up any replacement characters or noise
            clean_match = ''.join(c for c in match if ord(c) < 128 and c.isprintable() or c in ',.*$')
            if len(clean_match) > 50:  # Basic sanity check for GGA length
                gga_sentences.append(clean_match)
    
    except:
        # If decoding fails completely, try a different approach
        # Look for GGA patterns in the raw bytes
        try:
            # Convert to string representation and search
            str_data = str(data_chunk)
            if 'GGA' in str_data:
                # Extract potential GGA sentences using a simpler method
                parts = str_data.split('$')
                for part in parts:
                    if 'GGA' in part and ',' in part:
                        # Try to reconstruct a GGA sentence
                        gga_part = '$' + part.split('\\')[0]  # Remove any escape sequences
                        if len(gga_part) > 50:
                            gga_sentences.append(gga_part)
        except:
            pass
    
    return gga_sentences

def parse_gga_precision(gga_line):
    """Parse NMEA GGA sentence and extract coordinate precision."""
    try:
        gga_line = gga_line.strip()
        
        if not gga_line.startswith('$') or 'GGA' not in gga_line:
            return None
            
        # Remove checksum if present
        if '*' in gga_line:
            gga_line = gga_line.split('*')[0]
            
        # Split by comma and extract fields
        fields = gga_line[1:].split(',')
        
        if len(fields) < 9:
            return None
            
        # Extract coordinate fields
        time_field = fields[1] if len(fields) > 1 else ""
        lat_field = fields[2] if len(fields) > 2 else ""
        lat_dir = fields[3] if len(fields) > 3 else ""
        lon_field = fields[4] if len(fields) > 4 else ""
        lon_dir = fields[5] if len(fields) > 5 else ""
        quality = fields[6] if len(fields) > 6 else ""
        num_sats = fields[7] if len(fields) > 7 else ""
        hdop = fields[8] if len(fields) > 8 else ""
        
        if not lat_field or not lon_field:
            return None
            
        # Count decimal places
        lat_decimals = len(lat_field.split('.')[1]) if '.' in lat_field else 0
        lon_decimals = len(lon_field.split('.')[1]) if '.' in lon_field else 0
        
        # Convert to decimal degrees
        def nmea_to_decimal(coord_str, direction):
            if not coord_str or len(coord_str) < 4:
                return None
                
            try:
                # NMEA format: DDMM.MMMMM (lat) or DDDMM.MMMMM (lon)
                # Need to separate degrees and minutes properly
                
                if len(coord_str.split('.')[0]) == 4:  # Latitude: DDMM
                    degrees = float(coord_str[:2])
                    minutes = float(coord_str[2:])
                elif len(coord_str.split('.')[0]) == 5:  # Longitude: DDDMM  
                    degrees = float(coord_str[:3])
                    minutes = float(coord_str[3:])
                else:
                    return None
                    
                decimal_degrees = degrees + minutes / 60.0
                
                if direction in ['S', 'W']:
                    decimal_degrees = -decimal_degrees
                    
                return decimal_degrees
            except:
                return None
        
        lat_decimal = nmea_to_decimal(lat_field, lat_dir)
        lon_decimal = nmea_to_decimal(lon_field, lon_dir)
        
        return {
            'timestamp': time_field,
            'lat_raw': lat_field,
            'lat_dir': lat_dir,
            'lon_raw': lon_field,
            'lon_dir': lon_dir,
            'lat_decimal_places': lat_decimals,
            'lon_decimal_places': lon_decimals,
            'lat_decimal_degrees': lat_decimal,
            'lon_decimal_degrees': lon_decimal,
            'quality': quality,
            'num_satellites': num_sats,
            'hdop': hdop
        }
        
    except Exception as e:
        logging.debug(f"Error parsing GGA '{gga_line}': {e}")
        return None

def calculate_precision_meters(decimal_places):
    """Calculate precision in meters based on decimal places."""
    if decimal_places <= 0:
        return "No decimal precision"
    
    precision_minutes = 10 ** (-decimal_places)
    precision_meters = precision_minutes * 1852.0
    
    if precision_meters >= 1.0:
        return f"~{precision_meters:.1f} m"
    elif precision_meters >= 0.01:
        return f"~{precision_meters*100:.1f} cm"
    else:
        return f"~{precision_meters*1000:.1f} mm"

def monitor_gga_precision(ser, duration):
    """Monitor GGA messages in mixed RTCM/NMEA data stream."""
    logging.info(f"Monitoring GGA precision for {duration}s...")
    logging.info("Note: Parsing mixed RTCM/NMEA data stream...")
    
    start_time = time.time()
    gga_count = 0
    total_bytes = 0
    precision_stats = {}
    latest_position = None
    all_gga_sentences = []
    
    while time.time() - start_time < duration:
        if ser.in_waiting:
            # Read larger chunks to handle mixed data better
            data = ser.read(2048)
            total_bytes += len(data)
            
            # Extract GGA sentences from mixed data
            gga_sentences = extract_gga_from_mixed_data(data)
            
            for gga_line in gga_sentences:
                logging.debug(f"Extracted GGA: {gga_line}")
                
                gga_data = parse_gga_precision(gga_line)
                
                if gga_data and gga_data['lat_decimal_places'] > 0:
                    gga_count += 1
                    latest_position = gga_data
                    all_gga_sentences.append(gga_line)
                    
                    lat_prec = gga_data['lat_decimal_places']
                    lon_prec = gga_data['lon_decimal_places']
                    
                    key = f"{lat_prec},{lon_prec}"
                    precision_stats[key] = precision_stats.get(key, 0) + 1
                    
                    # Log first few messages
                    if gga_count <= 3:
                        logging.info(f"GGA #{gga_count}: {gga_line}")
                        logging.info(f"  Raw Lat: {gga_data['lat_raw']} {gga_data['lat_dir']} ({lat_prec} decimal places)")
                        logging.info(f"  Raw Lon: {gga_data['lon_raw']} {gga_data['lon_dir']} ({lon_prec} decimal places)")
                        logging.info(f"  Decimal Lat: {gga_data['lat_decimal_degrees']:.8f}")
                        logging.info(f"  Decimal Lon: {gga_data['lon_decimal_degrees']:.8f}")
                        logging.info(f"  Lat Precision: {calculate_precision_meters(lat_prec)}")
                        logging.info(f"  Lon Precision: {calculate_precision_meters(lon_prec)}")
                        logging.info(f"  Quality: {gga_data['quality']}, Satellites: {gga_data['num_satellites']}")
                        logging.info("")
        
        time.sleep(0.1)
    
    # Summary
    logging.info("=" * 60)
    logging.info("GGA PRECISION ANALYSIS SUMMARY")
    logging.info("=" * 60)
    logging.info(f"Total bytes processed: {total_bytes}")
    logging.info(f"Total GGA messages analyzed: {gga_count}")
    
    if latest_position:
        lat_prec = latest_position['lat_decimal_places']
        lon_prec = latest_position['lon_decimal_places']
        
        logging.info(f"\nLatest Position Analysis:")
        logging.info(f"  Raw Latitude:  {latest_position['lat_raw']} {latest_position['lat_dir']}")
        logging.info(f"  Raw Longitude: {latest_position['lon_raw']} {latest_position['lon_dir']}")
        logging.info(f"  Decimal Places: Lat={lat_prec}, Lon={lon_prec}")
        logging.info(f"  Decimal Coordinates: {latest_position['lat_decimal_degrees']:.8f}, {latest_position['lon_decimal_degrees']:.8f}")
        
        logging.info(f"\nPrecision Analysis:")
        logging.info(f"  Latitude precision:  {calculate_precision_meters(lat_prec)}")
        logging.info(f"  Longitude precision: {calculate_precision_meters(lon_prec)}")
        
        if precision_stats:
            logging.info(f"\nPrecision Distribution:")
            for prec_combo, count in sorted(precision_stats.items()):
                lat_p, lon_p = prec_combo.split(',')
                logging.info(f"  {lat_p},{lon_p} decimal places: {count} messages")
        
        logging.info(f"\nInterpretation:")
        if lat_prec >= 5:
            logging.info("  Position precision is EXCELLENT (< 2 cm) - RTK Fixed solution")
        elif lat_prec >= 4:
            logging.info("  Position precision is VERY GOOD (< 20 cm) - RTK Float solution")
        elif lat_prec >= 3:
            logging.info("  Position precision is GOOD (< 2 m) - Standard DGPS")
        elif lat_prec >= 2:
            logging.info("  Position precision is FAIR (< 20 m) - Basic GPS")
        else:
            logging.info("  Position precision is POOR (< 200 m) - Degraded GPS")
            
        # Show some sample GGA sentences
        if len(all_gga_sentences) > 0:
            logging.info(f"\nSample GGA sentences found:")
            for i, sentence in enumerate(all_gga_sentences[:3]):
                logging.info(f"  {i+1}: {sentence}")
                
    else:
        logging.warning("No valid GGA messages found in data stream")
        logging.info(f"Processed {total_bytes} bytes total")

def main():
    logging.info(f"Connecting to F9P on {PORT} @ {BAUDRATE}")
    logging.info("This script handles mixed RTCM/NMEA data streams")
    
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            ser.flushInput()
            time.sleep(1)
            monitor_gga_precision(ser, MONITOR_DURATION)
            
    except Exception as e:
        logging.error(f"Failed to connect or monitor: {e}")

if __name__ == "__main__":
    main()