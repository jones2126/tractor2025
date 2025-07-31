# RTK Base Station Setup

## Overview

This document describes a self-contained, solar-powered RTK (Real-Time Kinematic) base station that broadcasts RTCM (Radio Technical Commission for Maritime Services) correction data via TCP socket to a local network. The system is designed to use an F9P with minimal configuration changes, extract the RTCM messages and publish those to the network at 1Hz to prevent network congestion.

## System Components

### Power System
- **Solar Panel**: 100W photovoltaic panel for primary power generation
- **Charge Controller**: Renogy Wanderer 10 amp solar charge controller for battery management
- **Battery**: 12V lead-acid battery for energy storage and night/cloudy weather operation
- **DC-DC Converter**: TOBSUN EA15-5V 15W converter (12V to 5V) for Raspberry Pi power supply
- **ESP32 with TTL RS232 adapter**: Provides ability to interface and pull data from RJ12 port on Renogy controller

### Computing and GPS
- **Main Computer**: Raspberry Pi 3B running the base station python script publishing RTCM correction data
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

The python script, rtcm_server_0714.py, (aka RTCM server) posts to port 6001 **RTCM messages at 1Hz** to avoid network congestion:

- **F9P Output**: Generates RTCM messages at ~10 Hz per message type
- **Buffering System**: Latest message of each type is stored in memory buffer
- **Broadcast Thread**: Dedicated thread manages rate-limited transmission
- **Broadcast Rate**: Buffered messages are transmitted at **1 Hz** to connected clients
- **Data Rate**: Approximately 467 B/s (vs. 5.6 KB/s without rate limiting)

## Development and Deployment

### Git Sparse Checkout Configuration

This RPi 3, RTK base station, is configured with **Git Sparse Checkout** to only synchronize the `BridgevilleRTKBase/` folder from the main repository. This keeps the Pi's storage usage minimal and focuses only on the RTK base station functionality.

**Repository**: `https://github.com/jones2126/tractor2025`  
**Local Path**: `/home/al/python/tractor2025/BridgevilleRTKBase`  
**Sparse Checkout Pattern**: `BridgevilleRTKBase/*`  
**Active Branch**: `main`

### Development Workflow

1. **Primary Development**: Windows 10 laptop with VSCode
2. **Deployment**: Push changes to GitHub using VSCode functionality, then pull on Raspberry Pi
```bash
cd /home/al/python
git pull origin main
```

**Run python scripts**:
```bash
cd /home/al/python/BridgevilleRTKBase
python3 script_name.py
```

**Compile scripts for ESP32**:
```bash
cd /home/al/python/tractor2025/BridgevilleRTKBase/esp32/production/esp32-renogy-csv-logger
pio run --target upload
```
### Software Components

The system includes several Python scripts for different functionalities:

#### "RTCM Server" (rtcm_server_0714.py)
  - **Sets Broadcast Rate**: Sets RTCM publishing at 1Hz broadcast rate to limit network load
  - **Uses threading**: Separate threads for data receiving, broadcasting, and client handling instead of the main loop.  I had issues when I did not use threading.
  - **Reporting**: Tracks received and broadcast message rates
  - **RTCM Validation**: Uses CRC checking
  - **Logging**: Trying file rotation logic to manage space.  Early attempt.

#### Archived Scripts
- **Communication Scripts**: `com2_*.py` - Handle serial communication with RTK hardware
- **Logging Scripts**: `test*_logging.py` - Various logging implementations and tests  
- **RTCM Scripts**: `rtcm_*.py` - Handle RTCM correction data processing and serving
- **Configuration Scripts**: `configure_*.py`, `setup_*.sh` - Hardware and system configuration
- **Menu/Control Scripts**: `k706_menu*.py` - User interface for RTK configuration
- **logger_setup.py**: Reusable logging configuration module with file rotation

#### For Testing RTCM Messages - simple
- **rtcm_monitor_first_check.py**: Run this on a different computer that is also connected to your local network to confirm RTCM message rates, and connection status

#### For Testing if RTK Fix is Achieve
- **f9pGetRTCM.py**: Run this on a different computer that has an F9P GPS connected.  This will pass the RTCM messages to it and track if you are getting an RTK Fix status.

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

## Systemd Service Setup

To ensure the RTCM server starts automatically at boot and runs continuously, set up a systemd service:

### 1. Create the Service File

Create the service configuration file:

```bash
sudo nano /etc/systemd/system/rtcm_server.service
```

Add the following content:

```ini
[Unit]
Description=RTCM Base Station Server
After=network-online.target
Wants=network-online.target
StartLimitBurst=5
StartLimitIntervalSec=300

[Service]
Type=simple
User=al
Group=al
WorkingDirectory=/home/al/python/tractor2025/BridgevilleRTKBase/raspberry-pi/production
ExecStart=/usr/bin/python3 /home/al/python/tractor2025/BridgevilleRTKBase/raspberry-pi/production/rtcm_server_0714.py
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal
Environment=PYTHONPATH=/home/al/python/tractor2025/BridgevilleRTKBase/raspberry-pi/production

[Install]
WantedBy=multi-user.target
```

### 2. Enable and Start the Service

```bash
# Reload systemd to recognize the new service
sudo systemctl daemon-reload

# Enable the service to start at boot
sudo systemctl enable rtcm_server.service

# Start the service immediately
sudo systemctl start rtcm_server.service

# Check service status
sudo systemctl status rtcm_server.service
```

### 3. Setup UDEV for ESP32 and F9P

Create the udev file:

```bash
sudo nano /etc/udev/rules.d/99-rtk-devices.rules
```

Add the following content:

```ini
# UDEV rules for RTK Base Station devices /etc/udev/rules.d/99-rtk-devices.rules

# Creates /dev/esp32 symlink for the ESP32 with CH341 USB-to-Serial converter
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="esp32", GROUP="dialout", MODE="0664"

# Creates /dev/f9p symlink for the u-blox F9P GNSS receiver
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="f9p", GROUP="dialout", MODE="0664"

```

Reload UDEV rules:
```bash
bashsudo udevadm control --reload-rules
sudo udevadm trigger
```

Test by unplugging and reconnecting your devices

What These Rules Do:

ESP32: Creates /dev/esp32 symlink (instead of /dev/ttyUSB0)
F9P GPS: Creates /dev/f9p symlink (instead of /dev/ttyACM0)

### 3. Useful Commands

```bash
# Check service status
sudo systemctl status rtcm_server

# View service logs
sudo journalctl -u rtcm_server -f

# View recent logs
sudo journalctl -u rtcm_server --since "1 hour ago"

# Stop the service
sudo systemctl stop rtcm_server

# Restart the service
sudo systemctl restart rtcm_server

# Disable auto-start
sudo systemctl disable rtcm_server

# Check if the serial port is accessible
ls -la /dev/ttyACM0

# Verify user permissions
sudo usermod -a -G dialout al
```


## System Information

- **Hardware**: Raspberry Pi 3B (RTK Base Station)
- **OS**: Ubuntu 24.04 
- **Python Version**: Python 3.x
- **Primary Function**: RTK base station for precision GPS corrections
- **Network**: WiFi connected (IP: 192.168.1.233 - confirm - use ZeroTier VPN?)
- **Server Status**: TCP server on port 6001
- **Service Status**: Managed by /etc/systemd/system/rtcm_server.service

## Remote Access Tools

- **NoMachine**: For remote desktop access
- **FileZilla**: For file transfers  
- **PuTTY**: For SSH access
- **ZeroTier**: VPN for secure remote access

## Logging and Monitoring

### Log Files
- **Application Logs**: `logs/rtcm_server_0714.log` (with rotation)
- **GPS Raw Data**: `gps_log.txt` (binary format)
- **System Service Logs**: `sudo journalctl -u rtcm_server` (systemd journal)
- **Log Rotation**: 5MB files, 3 backups retained

### Real-time Monitoring
The system provides comprehensive real-time monitoring with enhanced statistics:

**Example Server Output (0714 version):**
```
=== Message Rates Report (over 10.0s) ===
RTCM Messages (Received from F9P):
  Type 1005: 11.19 Hz (112 messages)
  Type 1074: 9.20 Hz (92 messages)
  Type 1084: 9.20 Hz (92 messages)
  Type 1094: 9.20 Hz (92 messages)
  Type 1230: 9.20 Hz (92 messages)
Total: 47.99 Hz (480 messages)
TCP clients connected: 2
RTCM broadcasts: 5.00 Hz (50 message types sent)
==================================================
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

### Service Monitoring
```bash
# Monitor service in real-time
sudo journalctl -u rtcm_server -f

# Check service health
sudo systemctl is-active rtcm_server
sudo systemctl is-enabled rtcm_server

# View service restart history
sudo systemctl status rtcm_server
```

## Maintenance

- **Monthly**: Check battery voltage and connections
- **Quarterly**: Clean solar panel surface and inspect wiring
- **Annually**: Replace battery (lead-acid typical lifespan 3-5 years)
- **As Needed**: Software updates and log file management
- **Disk Space**: Monitor `/home/al/python/BridgevilleRTKBase/logs/` directory
- **Service Health**: Check `sudo systemctl status rtcm_server` periodically

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

### Service Issues
```bash
# Check if service is running
sudo systemctl status rtcm_server

# View service logs for errors
sudo journalctl -u rtcm_server --since "10 minutes ago"

# Restart the service
sudo systemctl restart rtcm_server

# Check serial port permissions
ls -la /dev/ttyACM0
sudo usermod -a -G dialout al
```

### Network Issues
1. **Check server status**: Look for "TCP server started on 0.0.0.0:6001" message in logs
2. **Test local connection**: `telnet localhost 6001` on the Pi
3. **firewall NOTE**: I chose not to setup ufw.  I did not feel the complexity offered any benefit
4. **Verify IP address**: `ip addr show wlan0` to confirm Pi's IP.  You can also logon to ZeroTier and see if an IP is assigned.

### System Performance
1. **Monitor RTCM rates**: Should see ~1 Hz output at the client end
2. **Check client connections**: Look for "TCP clients connected: X" in reports
3. **Verify GPS signals**: NMEA messages should show good satellite counts
4. **Service health**: `sudo systemctl is-active rtcm_server`

### If Git Operations Fail:
1. **Check disk space**: `df -h` (Pi storage fills up quickly)
2. **Clean up if needed**: 
   ```bash
   sudo apt clean
   sudo journalctl --vacuum-time=7d
   rm -f /home/al/python/BridgevilleRTKBase/gps_log.txt  # If very large
   ```

### RTCM Python Script Service Reset:
```bash
# If service is completely broken
sudo systemctl stop rtcm_server
sudo systemctl disable rtcm_server
sudo systemctl daemon-reload
sudo systemctl enable rtcm_server
sudo systemctl start rtcm_server
```


**Repository**: https://github.com/jones2126/tractor2025  
**Maintainer**: AL (aej2126 at protonmail dot com)  
**Last Updated**: July 2025