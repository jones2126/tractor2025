# RTK Base Station Setup

## Overview

This document describes a self-contained, solar-powered RTK (Real-Time Kinematic) base station that broadcasts RTCM (Radio Technical Commission for Maritime Services) correction data via TCP socket to a local network. The system is designed use an F9P with minimal configuration changes, extract the RTCM messages and publish those to the network at 1Hz to prevent network congestion.

## System Components

### Power System
- **Solar Panel**: 100W photovoltaic panel for primary power generation
- **Charge Controller**: Renogy Wanderer solar charge controller for battery management
- **Battery**: 12V lead-acid battery for energy storage and night/cloudy weather operation
- **DC-DC Converter**: TOBSUN EA15-5V 15W converter (12V to 5V) for Raspberry Pi power supply

### Computing and GPS
- **Main Computer**: Raspberry Pi 3B running the base station python script
- **GPS Receiver**: ArduSimple F9P high-precision GNSS module
- **Network Connection**: WiFi for local network connectivity

## RTCM Message Types

The F9P broadcasts the following RTCM 3.x correction messages:

| Message Type | Description | Typical Rate |
|--------------|-------------|--------------|
| **1005** | Stationary RTK reference station ARP (Antenna Reference Point) | 1 Hz |
| **1074** | GPS MSM4 (Multiple Signal Message) | 1 Hz |
| **1084** | GLONASS MSM4 | 1 Hz |
| **1094** | Galileo MSM4 | 1 Hz |
| **1230** | GLONASS code-phase biases | 1 Hz |

## Power Flow

```
Solar Panel (100W) → Charge Controller → 12V Battery
                                      ↓
                           DC-DC Converter (12V→5V)
                                      ↓
                              Raspberry Pi 3B
```

## Data Flow

```
GPS Satellites → F9P GPS Module → Raspberry Pi 3B → TCP Server → Local Network
                     (~10 Hz)         (Rate Limited)    (1 Hz)
```

## Rate Limiting and Network Optimization

The python script posts to port 6001 **RTCM messages at 1Hz** to avoid network congestion:

- **F9P Output**: Generates RTCM messages at ~10 Hz per message type
- **Buffering System**: Latest message of each type is stored in memory
- **Broadcast Rate**: Buffered messages are transmitted at **1 Hz** to all connected clients
- **Data Rate**: Approximately 467 B/s (vs. 5.6 KB/s without rate limiting)

## Development and Deployment

### Git Sparse Checkout Configuration

This RTK base station is configured with **Git Sparse Checkout** to only synchronize the `BridgevilleRTKBase/` folder from the main repository. This keeps the Pi's storage usage minimal and focuses only on the RTK base station functionality.

**Repository**: `https://github.com/jones2126/tractor2025`  
**Local Path**: `/home/al/python/`  
**Sparse Checkout Pattern**: `BridgevilleRTKBase/*`  
**Active Branch**: `main`

### Development Workflow

1. **Primary Development**: Windows 10 laptop with VSCode
2. **Deployment**: Push changes to GitHub using VSCode functionality, then pull on Raspberry Pi
```bash
cd /home/al/python
git pull origin main
```

**Run scripts**:
```bash
cd /home/al/python/BridgevilleRTKBase
python3 script_name.py
```

### Software Components

The system includes several Python scripts for different functionalities:

#### Core RTCM Server
- **rtcm_server_0713.py**: Main RTK base station server with:
  - RTCM message parsing and validation (CRC checking)
  - Rate-limited TCP broadcasting (1 Hz output from ~10 Hz input)
  - Multi-threaded client handling
  - Comprehensive logging with file rotation
  - Real-time message rate monitoring
  - Support for GPS, GLONASS, and Galileo constellations

#### Supporting Scripts
- **Communication Scripts**: `com2_*.py` - Handle serial communication with RTK hardware
- **Logging Scripts**: `test*_logging.py` - Various logging implementations and tests  
- **RTCM Scripts**: `rtcm_*.py` - Handle RTCM correction data processing and serving
- **Configuration Scripts**: `configure_*.py`, `setup_*.sh` - Hardware and system configuration
- **Menu/Control Scripts**: `k706_menu*.py` - User interface for RTK configuration
- **logger_setup.py**: Reusable logging configuration module with file rotation

#### Monitoring and Testing
- **rtcm_monitor.py**: Utility to run on a separate, network attached computer to check RTCM message rates, and connection status

### GPS Hardware Evolution

The current system uses the **ArduSimple ZED-F9P** GPS module, but previous iterations included:

- **Navspark PX1125R GNSS**: Earlier GPS receiver that was configured as an RTK base station with RTCM message output capabilities
- **ComNav K706 GPS**: Previous GPS module with configuration handled by the `k706_menu.py` script

The `k706_menu.py` script provides a menu-driven interface for configuring the K706 GPS with options for:
- Logging version information
- RTK base station reconfiguration
- Port configuration display
- Message monitoring
- Fixed position setting for RTK base
- Custom command sending

## System Features

- **Autonomous Operation**: Solar-powered system requires no external power source
- **Weather Resilient**: Battery backup ensures operation during low-light conditions
- **High Precision**: F9P module provides centimeter-level accuracy for RTK corrections
- **Network Integration**: TCP server enables real-time data distribution with rate limiting
- **Multi-Constellation**: Supports GPS, GLONASS, and Galileo satellite systems
- **Traffic Optimization**: Intelligent buffering prevents network congestion
- **Real-time Monitoring**: Comprehensive logging and rate reporting

## Technical Specifications

- **Power Consumption**: Approximately 10-15W continuous operation
- **Solar Generation**: Up to 100W peak power generation
- **GPS Accuracy**: <2cm horizontal, <3cm vertical (typical RTK base performance)
- **RTCM Update Rate**: 1 Hz broadcast rate (rate-limited from 10 Hz input)
- **Network Protocol**: TCP sockets (port 6001)
- **Operating Temperature**: -20°C to +70°C (component dependent)
- **Data Rate**: ~467 B/s (optimized for network efficiency)

## Installation Requirements

1. **Location**: Clear sky view with minimal obstructions (>15° elevation mask)
2. **Mounting**: Stable platform for GPS antenna and solar panel
3. **Network Access**: WiFi connection to local network
4. **Grounding**: Proper electrical grounding for lightning protection

## Network Configuration

The system creates a TCP server accessible on the local network, broadcasting rate-limited RTCM correction data in real-time. Client devices (rovers) can connect to this server to receive high-precision positioning corrections.

**Default Configuration:**
- **Port**: 6001
- **Protocol**: TCP sockets
- **Host**: 0.0.0.0 (all interfaces)
- **Data Format**: RTCM 3.x binary messages
- **Rate Limiting**: 1 Hz per message type
- **Authentication**: None (local network only)

**Connection Example:**
```bash
# From rover or monitoring device
telnet 192.168.1.233 6001
```

## System Information

- **Hardware**: Raspberry Pi 3B (RTK Base Station)
- **OS**: Ubuntu 24.04 
- **Python Version**: Python 3.x
- **Primary Function**: RTK base station for precision GPS corrections
- **Network**: WiFi connected (IP: 192.168.1.233)
- **Server Status**: TCP server on port 6001

## Remote Access Tools

- **NoMachine**: For remote desktop access
- **FileZilla**: For file transfers  
- **PuTTY**: For SSH access
- **ZeroTier**: VPN for secure remote access

## Logging and Monitoring

### Log Files
- **Application Logs**: `logs/rtcm_server_0713.log` (with rotation)
- **GPS Raw Data**: `gps_log.txt` (binary format)
- **Log Rotation**: 5MB files, 3 backups retained

### Real-time Monitoring
The system provides comprehensive real-time monitoring:

**Example Server Output:**
```
=== Message Rates Report (over 10.0s) ===
RTCM Messages (Received from F9P):
  Type 1005: 11.19 Hz (112 messages)
  Type 1074: 9.20 Hz (92 messages)
  Type 1084: 9.20 Hz (92 messages)
  Type 1094: 9.20 Hz (92 messages)
  Type 1230: 9.20 Hz (92 messages)
TCP clients connected: 1
RTCM broadcasts: 5.00 Hz (50 message types sent)
```

**Example Client Monitoring (using rtcm_monitor.py):**
```
📊 RTCM Statistics (last 10.0s):
   📈 Data Rate: 467.2 B/s
   📦 Message Types & Rates:
      1005:  1.00 Hz ( 10 msgs) - Base Station Antenna Position
      1074:  1.00 Hz ( 10 msgs) - GPS MSM4
      1084:  1.00 Hz ( 10 msgs) - GLONASS MSM4
      1094:  1.00 Hz ( 10 msgs) - Galileo MSM4
      1230:  1.00 Hz ( 10 msgs) - GLONASS Code-Phase Biases
   🔄 Total: 5.00 Hz (50 messages)
```

## Maintenance

- **Monthly**: Check battery voltage and connections
- **Quarterly**: Clean solar panel surface and inspect wiring
- **Annually**: Replace battery (lead-acid typical lifespan 3-5 years)
- **As Needed**: Software updates and log file management
- **Disk Space**: Monitor `/home/al/python/BridgevilleRTKBase/logs/` directory

## Project Repository GitHub Notes

### Files to Ignore (already in .gitignore):
- `logs/` - Runtime log files
- `*.log` - Individual log files  
- `*.bin` - Binary data files
- `gps_log.txt` - Raw GPS data file
- Any temporary or generated files

## Troubleshooting

### Git Configuration
You can verify the sparse checkout configuration:
```bash
cd /home/al/python
git config core.sparseCheckout  # Should show 'true'
cat .git/info/sparse-checkout    # Should show 'BridgevilleRTKBase/*'
```

### Network Issues
1. **Check server status**: Look for "TCP server started on 0.0.0.0:6001" message
2. **Test local connection**: `telnet localhost 6001` on the Pi
3. **Check firewall**: `sudo ufw status` and `sudo ufw allow 6001` if needed
4. **Verify IP address**: `ip addr show wlan0` to confirm Pi's IP

### System Performance
1. **Monitor RTCM rates**: Should see ~1 Hz output, ~10 Hz input
2. **Check client connections**: Look for "TCP clients connected: X" in reports
3. **Verify GPS signals**: NMEA messages should show good satellite counts

### If Git Operations Fail:
1. **Check disk space**: `df -h` (Pi storage fills up quickly)
2. **Clean up if needed**: 
   ```bash
   sudo apt clean
   sudo journalctl --vacuum-time=7d
   rm -f /home/al/python/BridgevilleRTKBase/gps_log.txt  # If very large
   ```

**Repository**: https://github.com/jones2126/tractor2025  
**Maintainer**: AL (aej2126 at protonmail dot com)  
**Last Updated**: July 2025