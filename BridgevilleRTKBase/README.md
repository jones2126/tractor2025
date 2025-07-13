# BridgevilleRTKBase - RTK Base Station Scripts

This directory contains Python scripts and configuration files for operating the RTK base station on the Raspberry Pi.

## Git Sparse Checkout Configuration

This Raspberry Pi is configured with **Git Sparse Checkout** to only synchronize the `BridgevilleRTKBase/` folder from the main repository. This keeps the Pi's storage usage minimal and focuses only on the RTK base station functionality.

### What is Sparse Checkout?
Sparse checkout allows you to work with only a subset of files from a Git repository. Instead of downloading the entire repository (which includes teensy code, documentation, and other components), this Pi only pulls down the RTK base station related files.

### Current Configuration
- **Repository**: `https://github.com/jones2126/tractor2025`
- **Local Path**: `/home/al/python/`
- **Sparse Checkout Pattern**: `BridgevilleRTKBase/*`
- **Active Branch**: `main`

You can verify the sparse checkout configuration:
```bash
cd /home/al/python
git config core.sparseCheckout  # Should show 'true'
cat .git/info/sparse-checkout    # Should show 'BridgevilleRTKBase/*'
```

## Development Workflow

### 1. Development on Windows 10 Laptop (Primary Development)

**Setup** (one-time):
```bash
# Clone the full repository on your Windows laptop
git clone https://github.com/jones2126/tractor2025.git
cd tractor2025
```

**Daily workflow**:
1. **Edit**: Use VSCode to edit Python scripts in `BridgevilleRTKBase/` folder
2. **Test locally** (if possible): Test scripts on your laptop if you have Python installed
3. **Commit changes**:
   ```bash
   git add BridgevilleRTKBase/
   git commit -m "Description of changes to RTK scripts"
   git push origin main
   ```

### 2. Deployment to Raspberry Pi

**Update the Pi** (after pushing changes from laptop):
```bash
cd /home/al/python
git pull origin main
```

**Run scripts**:
```bash
cd /home/al/python/BridgevilleRTKBase
python3 script_name.py
```

### 3. Emergency Edits on Pi

If you need to make quick fixes directly on the Pi:

```bash
cd /home/al/python/BridgevilleRTKBase
# Edit file with nano or vim
nano script_name.py

# Commit and push changes
cd /home/al/python
git add BridgevilleRTKBase/script_name.py
git commit -m "Quick fix: description of change"
git push origin main
```

**Important**: After pushing from Pi, remember to pull on your laptop:
```bash
# On Windows laptop
git pull origin main
```

## File Organization

### Python Scripts in this Directory:
- **Communication Scripts**: `com2_*.py` - Handle serial communication with RTK hardware
- **Logging Scripts**: `test*_logging.py` - Various logging implementations and tests
- **RTCM Scripts**: `rtcm_*.py` - Handle RTCM correction data processing and serving
- **Configuration Scripts**: `configure_*.py`, `setup_*.sh` - Hardware and system configuration
- **Menu/Control Scripts**: `k706_menu*.py` - User interface for RTK configuration
- **rtcm_server_0507.py**: An RTK base station server that configures a PX1125R GNSS receiver as an RTK base station, reads RTCM correction data from the serial interface, and broadcasts it via TCP. The script  configures the base station with coordinates to a location in Pennsylvania, sets up RTCM message output (including MSM4 messages), and continuously streams the correction data. It has logging with automatic log rotation and compression, real-time message rate monitoring and CRC error detection, RTCM message parsing (with special handling for 1005 station reference messages), and multi-threaded architecture to handle multiple simultaneous client connections. The server logs all RTCM data to timestamped files, provides detailed statistics on message types and transmission rates, and includes comprehensive error handling for both serial communication and network operations for operating an RTK base station.

### Files to Ignore (already in .gitignore):
- `logs/` - Runtime log files
- `*.log` - Individual log files  
- `*.bin` - Binary data files
- Any temporary or generated files

## Troubleshooting

### If Git Operations Fail:
1. **Check disk space**: `df -h` (Pi storage fills up quickly)
2. **Clean up if needed**: 
   ```bash
   sudo apt clean
   sudo journalctl --vacuum-time=7d
   ```

### If Sparse Checkout Stops Working:
```bash
cd /home/al/python
git config core.sparseCheckout true
echo "BridgevilleRTKBase/*" > .git/info/sparse-checkout
git read-tree -m -u HEAD
```

### To See Full Repository (temporarily):
```bash
git config core.sparseCheckout false
git read-tree -m -u HEAD
# When done, re-enable sparse checkout with commands above
```

## System Information

- **Hardware**: Raspberry Pi (RTK Base Station)
- **OS**: Ubuntu 24.04 
- **Python Version**: Python 3.x
- **Primary Function**: RTK base station for precision GPS corrections
- **Network**: WiFi connected for internet RTK corrections and remote access

## Remote Access Tools

- **NoMachine**: For remote desktop access
- **FileZilla**: For file transfers  
- **PuTTY**: For SSH access
- **ZeroTier**: VPN for secure remote access

---

**Last Updated**: July 2025  
**Maintainer**: AL (aej2126 at protonmail dot com)