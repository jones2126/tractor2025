# ESP32 Renogy Data Logger

A data logging system that monitors a solar powered GPS RTK base station.  Current data includes temperature and Renogy charge controller status.  A Renogy Wanderer 10A controllers, an ESP32, and a Raspberry Pi 3 B are components in the system.

## System Overview

Logical components:
1. **ESP32 Logger** - Collects temperature and charge controller data to SPIFFS
2. **Python Download Script** - Retrieves data daily from ESP32 via USB serial
3. **Automated Cron Job** - Controls the collection frequency

## Hardware Requirements

- **ESP32 Development Board** (ESP32-DevKitM-1 30-pin)
- **DS18B20 Temperature Sensor** (connected to GPIO32)
- **4.7K resistor** (connects between VCC and GPIO32)
- **Renogy 10 amp Charge Controller** with RS232/Modbus capability
- **RS232 to TTL Converter** (for RJ12 connection to Renogy controller)
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

## ESP32 Script

### Data Collection Timing
- **Temperature**: Read every 15 seconds, averaged over 1-minute periods
- **Renogy Data**: Read every 20 seconds  
- **CSV Logging**: Write averaged data every 60 seconds to ESP32 SPIFFS

### ESP32 Storage Management
- **File Size Limit**: 500KB per CSV file
- **Auto-Rotation**: Creates new files when size limit reached
- **Naming Convention**: `/data_log_1.csv`, `/data_log_2.csv`, etc.

### Data Collected
Each CSV row contains:
- Timestamp (milliseconds since boot) - need to add code to get actual date/time
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

### Usage

**Typical Commands:**
```bash
# Check ESP32 status
python3 esp32_downloader.py status

# Download data (keep on ESP32)
python3 esp32_downloader.py download [optional_filename]

# Download and delete (for daily automation)
python3 esp32_downloader.py download_delete [optional_filename]

# Interactive mode (i.e. direct control in terminal window)
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

1. **Create data directory and script file:**
```bash
mkdir -p /home/al/esp32_data
mkdir -p /home/al/logs
mkdir -p /home/al/scripts
nano /home/al/scripts/daily_esp32_download.sh
Copy and paste script below
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

### Cron Job Description
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
│   ├── esp32-downloader.py          # Main download script
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

# Check ESP32 files
python3 /home/al/python/esp32-downloader.py status

# Test manual download
python3 /home/al/python/esp32-downloader.py download 
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

## Storage Capacity

- **ESP32 SPIFFS**: ~1.26 MB total storage
- **File rotation**: At 500KB per file (≈5-7 days per file)

## Development Notes

### Future Potential Enhancements
- Use WiFi connectivity for control and status  
- Database integration (SQLite/PostgreSQL) for data analysis
- Email alerts for system issues
- Additional sensor support (e.g. light diode)
