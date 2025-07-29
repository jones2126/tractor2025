# ESP32 Renogy Data Logger

A comprehensive data logging system that monitors temperature and Renogy charge controller data using an ESP32, with automated data collection and management.

## System Overview

This system consists of three main components:
1. **ESP32 Data Logger** - Collects temperature and Renogy charge controller data
2. **Python Download Script** - Retrieves data from ESP32 via USB serial
3. **Automated Cron Job** - Daily data collection and management

## Hardware Requirements

- **ESP32 Development Board**
- **DS18B20 Temperature Sensor** (connected to GPIO32)
- **Renogy Charge Controller** with RS232/Modbus capability
- **RS232 to TTL Converter** (for Renogy communication)
- **USB Cable** (ESP32 to Raspberry Pi connection)

### Pin Connections
```
ESP32 Pin    | Component           | Description
-------------|--------------------|---------------------------------
GPIO32       | DS18B20 Data       | Temperature sensor data line
GPIO16 (RX2) | RS232 TTL RX       | Renogy communication receive
GPIO17 (TX2) | RS232 TTL TX       | Renogy communication transmit
USB          | Raspberry Pi       | Data download and monitoring
```

## ESP32 Firmware Features

### Data Collection Timing
- **Temperature**: Read every 15 seconds, averaged over 1-minute periods
- **Renogy Data**: Read every 20 seconds  
- **CSV Logging**: Write averaged data every 60 seconds

### Automatic File Management
- **File Size Limit**: 500KB per CSV file
- **Auto-Rotation**: Creates new files when size limit reached
- **Naming Convention**: `/data_log_1.csv`, `/data_log_2.csv`, etc.

### Data Collected
Each CSV row contains:
- Timestamp (milliseconds since boot)
- Average temperature (°C and °F)
- Battery voltage, SOC, charging amps
- Solar panel voltage, amps, watts
- Controller and battery temperatures
- Load voltage, amps, watts

### Serial Commands
The ESP32 responds to these commands via USB serial:

| Command | Description |
|---------|-------------|
| `STATUS` | Show current file info and storage status |
| `DOWNLOAD` | Download current CSV file (keep on ESP32) |
| `DOWNLOAD_DELETE` | Download CSV file and delete from ESP32 |
| `CLEAR` | Delete current CSV file and start fresh |
| `HELP` | Show available commands |

## Python Download Script

### Installation
```bash
# Install required Python package
pip3 install pyserial

# Make script executable
chmod +x esp32_downloader.py
```

### Usage

**Command Line Options:**
```bash
# Check ESP32 status
python3 esp32_downloader.py status

# Download data (keep on ESP32)
python3 esp32_downloader.py download [optional_filename]

# Download and delete (for daily automation)
python3 esp32_downloader.py download_delete [optional_filename]

# Interactive mode
python3 esp32_downloader.py
```

**Interactive Mode Commands:**
- `download [filename]` - Download data, keep on ESP32
- `download_delete [filename]` - Download and delete from ESP32
- `status` - Show file status
- `clear` - Clear ESP32 data
- `help` - Show ESP32 help
- `quit` - Exit

### File Naming
Downloaded files are automatically named with timestamps:
```
esp32_data_20250729_141500.csv
```

## Automated Daily Collection

### Cron Job Setup

1. **Create data directory:**
```bash
mkdir -p /home/al/esp32_data
```

2. **Create daily download script:**
```bash
#!/bin/bash
# /home/al/scripts/daily_esp32_download.sh

DATE=$(date +%Y%m%d)
DATA_DIR="/home/al/esp32_data"
SCRIPT_DIR="/home/al/python"

cd $SCRIPT_DIR
python3 esp32_downloader.py download_delete "$DATA_DIR/esp32_data_$DATE.csv"

# Optional: Compress old files (older than 30 days)
find $DATA_DIR -name "*.csv" -mtime +30 -exec gzip {} \;

# Optional: Delete very old files (older than 365 days)
find $DATA_DIR -name "*.csv.gz" -mtime +365 -delete
```

3. **Make script executable:**
```bash
chmod +x /home/al/scripts/daily_esp32_download.sh
```

4. **Add to crontab:**
```bash
crontab -e
```

Add this line for daily 6 AM collection:
```cron
0 6 * * * /home/al/scripts/daily_esp32_download.sh >> /home/al/logs/esp32_cron.log 2>&1
```

### Cron Job Features
- **Daily automated download** at 6 AM
- **Automatic file deletion** from ESP32 after successful download
- **Timestamped filenames** for easy organization
- **Log file** for troubleshooting
- **Optional compression** of old files to save space
- **Automatic cleanup** of very old files

## File Structure

```
/home/al/
├── python/
│   ├── esp32_downloader.py          # Main download script
│   └── requirements.txt             # Python dependencies
├── scripts/
│   └── daily_esp32_download.sh      # Daily cron script
├── esp32_data/                      # Downloaded CSV files
│   ├── esp32_data_20250729.csv
│   ├── esp32_data_20250730.csv
│   └── ...
└── logs/
    └── esp32_cron.log              # Cron job logs
```

## Monitoring and Troubleshooting

### Check System Status
```bash
# View recent cron job logs
tail -f /home/al/logs/esp32_cron.log

# Test manual download
python3 esp32_downloader.py status

# Check ESP32 files
python3 esp32_downloader.py status
```

### Common Issues

**"Multiple access on port" error:**
- Close PlatformIO monitor before running Python script
- Only one process can access the serial port at a time

**ESP32 not responding:**
- Check USB connection
- Verify correct port (usually `/dev/ttyUSB0`)
- Reset ESP32 if needed

**Cron job not running:**
- Check cron service: `sudo systemctl status cron`
- Verify crontab entry: `crontab -l`
- Check log file for errors

## Data Analysis

The CSV files can be imported into various tools:

**Python/Pandas:**
```python
import pandas as pd
df = pd.read_csv('esp32_data_20250729.csv')
```

**Excel/LibreOffice:**
- Direct CSV import with comma delimiter

**InfluxDB/Grafana:**
- For time-series visualization and monitoring

## Storage Capacity

- **ESP32 SPIFFS**: ~1.26 MB total storage
- **File rotation**: At 500KB per file (≈5-7 days per file)
- **Daily downloads**: Prevents storage overflow
- **Raspberry Pi**: Virtually unlimited storage for historical data

## Development Notes

### ESP32 Code Structure
- `platformio.ini` - Library dependencies
- `src/main.cpp` - Main firmware code
- Libraries: OneWire, DallasTemperature, ModbusMaster

### Python Dependencies
```txt
pyserial>=3.4
```

### Future Enhancements
- WiFi connectivity for remote downloads
- Web dashboard for real-time monitoring  
- Database integration (SQLite/PostgreSQL)
- Email alerts for system issues
- Additional sensor support

## License

This project is released under the MIT License. Feel free to modify and distribute as needed.

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review log files for error messages
3. Test components individually (ESP32, Python script, cron job)
4. Ensure all hardware connections are secure