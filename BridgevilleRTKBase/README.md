# RTK Base Station Setup

## Overview

This document describes a self-contained, solar-powered RTK (Real-Time Kinematic) base station that broadcasts RTCM (Radio Technical Commission for Maritime Services) correction data via WebSocket to a local network. The system is designed for autonomous operation with minimal maintenance requirements.

## System Components

### Power System
- **Solar Panel**: 100W photovoltaic panel for primary power generation
- **Charge Controller**: Renogy Wanderer solar charge controller for battery management
- **Battery**: 12V lead-acid battery for energy storage and night/cloudy weather operation
- **DC-DC Converter**: TOBSUN EA15-5V 15W converter (12V to 5V) for Raspberry Pi power supply

### Computing and GPS
- **Main Computer**: Raspberry Pi 3B running the base station software
- **GPS Receiver**: ArduSimple F9P high-precision GNSS module
- **Network Connection**: Ethernet/WiFi for local network connectivity

## RTCM Message Types

The base station broadcasts the following RTCM 3.x correction messages:

| Message Type | Description |
|--------------|-------------|
| **1005** | Stationary RTK reference station ARP (Antenna Reference Point) |
| **1074** | GPS MSM4 (Multiple Signal Message) |
| **1084** | GLONASS MSM4 |
| **1094** | Galileo MSM4 |
| **1230** | GLONASS code-phase biases |

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
GPS Satellites → F9P GPS Module → Raspberry Pi 3B → WebSocket Server → Local Network
```

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

- **Communication Scripts**: `com2_*.py` - Handle serial communication with RTK hardware
- **Logging Scripts**: `test*_logging.py` - Various logging implementations and tests  
- **RTCM Scripts**: `rtcm_*.py` - Handle RTCM correction data processing and serving
- **Configuration Scripts**: `configure_*.py`, `setup_*.sh` - Hardware and system configuration
- **Menu/Control Scripts**: `k706_menu*.py` - User interface for RTK configuration
- **rtcm_server_0507.py**: Main RTK base station server with RTCM processing, TCP broadcasting, logging with rotation, and multi-threaded client handling

### GPS Hardware Evolution

The current system uses the **ArduSimple F9P** GPS module, but previous iterations included:

- **Navspark PX1125R GNSS**: Earlier GPS receiver that was configured as an RTK base station with RTCM message output capabilities
- **ComNav K706 GPS**: Previous GPS module with configuration handled by the `k706_menu.py` script, which provided a command-line interface for GPS configuration including RTK base station setup, position fixing, and port configurations

The `k706_menu.py` script provides a menu-driven interface for configuring the K706 GPS with options for:
- Logging version information
- RTK base station reconfiguration
- Port configuration display
- Message monitoring
- Fixed position setting for RTK base
- Custom command sending

- **Autonomous Operation**: Solar-powered system requires no external power source
- **Weather Resilient**: Battery backup ensures operation during low-light conditions
- **High Precision**: F9P module provides centimeter-level accuracy for RTK corrections
- **Network Integration**: WebSocket server enables real-time data distribution
- **Multi-Constellation**: Supports GPS, GLONASS, and Galileo satellite systems

## Technical Specifications

- **Power Consumption**: Approximately 10-15W continuous operation
- **Solar Generation**: Up to 100W peak power generation
- **GPS Accuracy**: <2cm horizontal, <3cm vertical (typical RTK base performance)
- **Update Rate**: 1Hz RTCM message broadcast rate
- **Network Protocol**: WebSocket over TCP/IP
- **Operating Temperature**: -20°C to +70°C (component dependent)

## Installation Requirements

1. **Location**: Clear sky view with minimal obstructions (>15° elevation mask)
2. **Mounting**: Stable platform for GPS antenna and solar panel
3. **Network Access**: WiFi or Ethernet connection to local network
4. **Grounding**: Proper electrical grounding for lightning protection

## Maintenance

- **Monthly**: Check battery voltage and connections
- **Quarterly**: Clean solar panel surface and inspect wiring
- **Annually**: Replace battery (lead-acid typical lifespan 3-5 years)
- **As Needed**: Software updates and log file management

## Network Configuration

The system creates a WebSocket server accessible on the local network, broadcasting RTCM correction data in real-time. Client devices (rovers) can connect to this server to receive high-precision positioning corrections.

**Default Configuration:**
- Port: 8080 (configurable)
- Protocol: WebSocket (ws://)
- Data Format: RTCM 3.x binary messages
- Authentication: None (local network only)

## System Information

- **Hardware**: Raspberry Pi 3B (RTK Base Station)
- **OS**: Ubuntu 24.04 
- **Python Version**: Python 3.x
- **Primary Function**: RTK base station for precision GPS corrections
- **Network**: WiFi connected for internet RTK corrections and remote access

## Remote Access Tools

- **NoMachine**: For remote desktop access
- **FileZilla**: For file transfers  
- **PuTTY**: For SSH access
- **ZeroTier**: VPN for secure remote access

## Project Repository GitHub Notes

### Files to Ignore (already in .gitignore):
- `logs/` - Runtime log files
- `*.log` - Individual log files  
- `*.bin` - Binary data files
- Any temporary or generated files

## Troubleshooting

You can verify the sparse checkout configuration:
```bash
cd /home/al/python
git config core.sparseCheckout  # Should show 'true'
cat .git/info/sparse-checkout    # Should show 'BridgevilleRTKBase/*'
```

### If Git Operations Fail:
1. **Check disk space**: `df -h` (Pi storage fills up quickly)
2. **Clean up if needed**: 
   ```bash
   sudo apt clean
   sudo journalctl --vacuum-time=7d
   ```

**Repository**: https://github.com/jones2126/tractor2025  
**Maintainer**: AL (aej2126 at protonmail dot com)  
**Last Updated**: July 2025