# Teensy Serial Bridge Documentation

## Overview

The Teensy Serial Bridge (`teensy_serial_bridge.py`) is a Python application that runs on the Raspberry Pi 5 to bridge serial communication from the Teensy 3.5 microcontroller to UDP broadcasts for system monitoring and control.

**Primary Functions:**
1. Reads serial data from Teensy microcontroller
2. Parses structured messages from multiple subsystems
3. Broadcasts consolidated status via UDP at 5 Hz
4. Provides real-time monitoring of steering, transmission, and radio systems

**Version:** 1.0  
**Author:** Tractor2025 Project  
**Last Updated:** January 2025

---

## System Architecture

```
┌─────────────────┐
│  Teensy 3.5     │
│  (Hardware      │ Serial    ┌─────────────────────┐
│   Control)      │──USB──────┤ Raspberry Pi 5      │
│                 │ 115200    │ teensy_serial_bridge│
└─────────────────┘           │                     │
                              └──────────┬──────────┘
                                         │ UDP Broadcast
                                         │ Port 6003
                                         │ 5 Hz
                                         ▼
                              ┌─────────────────────┐
                              │ Monitoring Clients  │
                              │ (GUI, Logging, etc) │
                              └─────────────────────┘
```

---

## Message Format

### Teensy → Bridge (Serial)

The Teensy sends messages in this CSV format:

```
<msg_type>,<timestamp>,<subsystem>,<key1>=<value1>,<key2>=<value2>,...
```

**Fields:**
- `msg_type`: Integer message type (1=status, 2=event)
- `timestamp`: Milliseconds since Teensy boot
- `subsystem`: System identifier (RADIO, STEER, TRANS, SYSTEM)
- `data`: Comma-separated key=value pairs

**Examples:**

```
1,45678,RADIO,signal=GOOD,ack_rate=0.95,current_rate=4.8
1,45690,STEER,mode=2,setpt=0.5,current=0.48,error=0.02,dir=LEFT,pwm=45
1,45702,TRANS,mode=2,bucket=7,target=3200,current=3185
1,45714,SYSTEM,heartbeat=1
```

### Bridge → Monitors (UDP)

The bridge broadcasts JSON-formatted status at 5 Hz:

```json
{
    "timestamp": 1234567890.123,
    "source": "teensy_bridge",
    "radio": {
        "signal": "GOOD",
        "ack_rate": 0.95,
        "current_rate": 4.8,
        "age": 0.05
    },
    "steering": {
        "mode": 2,
        "setpoint": 0.5,
        "current": 0.48,
        "error": 0.02,
        "direction": "LEFT",
        "pwm": 45,
        "age": 0.05
    },
    "transmission": {
        "mode": 2,
        "bucket": 7,
        "target": 3200,
        "current": 3185,
        "age": 0.05
    },
    "system": {
        "heartbeat_age": 0.05
    }
}
```

---

## Configuration

### Serial Connection

```python
SERIAL_PORT = '/dev/teensy'  # Teensy USB device
BAUD_RATE = 115200          # Must match Teensy configuration
```

**udev Rule** (from your project):
```bash
# /etc/udev/rules.d/99-teensy.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", SYMLINK+="teensy"
```

### UDP Broadcast

```python
UDP_BROADCAST_IP = '255.255.255.255'  # Broadcast to all interfaces
UDP_PORT = 6003                        # UDP port for status broadcasts
BROADCAST_RATE = 5                     # Hz (5 broadcasts per second)
```

---

## Data Storage and Defaults

### Internal Data Structure

The bridge maintains the latest data from each subsystem:

```python
self.latest_data = {
    'RADIO': {
        'signal': 'GOOD',
        'ack_rate': 0.95,
        'current_rate': 4.8,
        'last_update': 1234567890.123
    },
    'STEER': {
        'mode': 2,
        'setpt': 0.5,
        'current': 0.48,
        'error': 0.02,
        'dir': 'LEFT',
        'pwm': 45,
        'last_update': 1234567890.124
    },
    'TRANS': {
        'mode': 2,
        'bucket': 7,
        'target': 3200,
        'current': 3185,
        'last_update': 1234567890.125
    }
}
```

### Default Values

The `.get()` method provides safe fallback values when data hasn't been received yet:

| Subsystem | Field | Default | Meaning |
|-----------|-------|---------|---------|
| TRANS | mode | 0 | Manual mode (before data received) |
| TRANS | bucket | 5 | Neutral position |
| TRANS | target | 2985 | Neutral servo position |
| TRANS | current | 2985 | Neutral servo position |
| STEER | mode | 0 | Manual mode |
| STEER | setpoint | 0.0 | Straight ahead |
| STEER | current | 0.0 | Straight ahead |
| STEER | error | 0.0 | No error |
| STEER | pwm | 0.0 | No motor power |
| RADIO | ack_rate | 0.0 | No acknowledgments |
| RADIO | current_rate | 0.0 | No messages |

**Important:** These are **fallback defaults**, not hard-coded values. Once the Teensy sends data, these are replaced with actual real-time values.

---

## Data Flow Example

### Step 1: Teensy Sends Serial Data

```cpp
// Teensy code sends:
Serial.print("1,");           // msg_type
Serial.print(millis());       // timestamp
Serial.print(",TRANS,mode="); 
Serial.print(2);
Serial.print(",bucket=");
Serial.print(7);
Serial.print(",target=");
Serial.print(3200);
Serial.print(",current=");
Serial.println(3185);

// Results in serial line:
// "1,45702,TRANS,mode=2,bucket=7,target=3200,current=3185"
```

### Step 2: Bridge Parses Message

```python
parsed = {
    'subsystem': 'TRANS',
    'data': {
        'msg_type': 1,
        'timestamp': 45702,
        'mode': 2,
        'bucket': 7,
        'target': 3200,
        'current': 3185
    }
}
```

### Step 3: Bridge Updates Internal Storage

```python
self.latest_data['TRANS'] = {
    'mode': 2,
    'bucket': 7,
    'target': 3200,
    'current': 3185,
    'last_update': 1234567890.125  # Pi's timestamp
}
```

### Step 4: Bridge Creates Broadcast Message

```python
message['transmission'] = {
    'mode': int(trans_data.get('mode', 0)),      # → 2
    'bucket': int(trans_data.get('bucket', 5)),  # → 7
    'target': trans_data.get('target', 2985),    # → 3200
    'current': trans_data.get('current', 2985),  # → 3185
    'age': current_time - trans_data.get('last_update', current_time)  # → 0.05
}
```

### Step 5: Bridge Broadcasts UDP Packet

```json
{
    "timestamp": 1234567890.175,
    "source": "teensy_bridge",
    "transmission": {
        "mode": 2,
        "bucket": 7,
        "target": 3200,
        "current": 3185,
        "age": 0.05
    }
}
```

---

## Usage

### Starting the Bridge

```bash
# Make sure Teensy is connected
ls -l /dev/teensy  # Should show the symlink

# Start the bridge
cd /path/to/tractor2025
python3 teensy_serial_bridge.py
```

### Expected Output

```
2025-01-15 10:30:45 - INFO - Serial connected to /dev/teensy at 115200 baud
2025-01-15 10:30:45 - INFO - UDP broadcast configured on port 6003
2025-01-15 10:30:45 - INFO - Teensy Serial Bridge starting...
2025-01-15 10:30:45 - INFO - Broadcasting on UDP port 6003 at 5 Hz
2025-01-15 10:30:50 - INFO - Broadcast #25: Radio=GOOD, Steer_mode=2, Trans_mode=2
2025-01-15 10:31:15 - INFO - Statistics - Received: 1523, Parsed: 1520, Broadcasts: 150, Errors: 0
```

### Running as a Service

Create systemd service file:

```bash
sudo nano /etc/systemd/system/teensy-bridge.service
```

```ini
[Unit]
Description=Teensy Serial Bridge
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/tractor2025
ExecStart=/usr/bin/python3 /home/pi/tractor2025/teensy_serial_bridge.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable and start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable teensy-bridge.service
sudo systemctl start teensy-bridge.service
sudo systemctl status teensy-bridge.service
```

---

## Monitoring UDP Broadcasts

### Using netcat

```bash
# Listen for UDP broadcasts
nc -ul 6003
```

### Using Python

```python
import socket
import json

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 6003))

while True:
    data, addr = sock.recvfrom(4096)
    message = json.loads(data.decode())
    print(f"Radio: {message['radio']['signal']}, "
          f"Steer Mode: {message['steering']['mode']}, "
          f"Trans Bucket: {message['transmission']['bucket']}")
```

### Using GUI Monitor (Recommended)

A separate GUI monitor application can subscribe to these broadcasts for real-time visualization. The UDP format makes it easy to develop custom monitoring tools.

---

## Subsystem Details

### RADIO Subsystem

**Purpose:** Monitor NRF24 radio communication health

**Data Fields:**
- `signal`: String - "GOOD", "WEAK", "LOST"
- `ack_rate`: Float - Acknowledgment success rate (0.0-1.0)
- `current_rate`: Float - Messages per second
- `age`: Float - Seconds since last update

**Typical Values:**
- Good operation: signal="GOOD", ack_rate > 0.90
- Warning: signal="WEAK", ack_rate 0.70-0.90
- Critical: signal="LOST", ack_rate < 0.70

### STEER Subsystem

**Purpose:** Monitor steering control system

**Data Fields:**
- `mode`: Integer - 0=Manual, 1=Assisted, 2=Autonomous
- `setpoint`: Float - Target steering angle (-0.96 to +0.96)
- `current`: Float - Actual steering angle (-0.96 to +0.96)
- `error`: Float - Difference between setpoint and current
- `direction`: String - "LEFT", "RIGHT", "CENTER"
- `pwm`: Integer - Motor PWM value (0-255)
- `age`: Float - Seconds since last update

**Angle Convention:**
- Negative values: Turn right
- Positive values: Turn left
- Zero: Straight ahead

### TRANS Subsystem

**Purpose:** Monitor transmission control system

**Data Fields:**
- `mode`: Integer - 0=Manual, 1=Auto-Speed, 2=Auto-Navigation
- `bucket`: Integer - Speed bucket (0-10, 5=neutral)
- `target`: Integer - Target servo position (245-325)
- `current`: Integer - Current servo position (245-325)
- `age`: Float - Seconds since last update

**Servo Position Map:**
- 245: Full reverse
- 266: Neutral
- 325: Full forward

**Bucket System:**
```
Bucket  Speed       Servo Position
------  ----------  --------------
0       Full Rev    245
1       75% Rev     258
2       Neutral     266
3-4     Reserved    
5       Neutral     266 (default)
6       25% Fwd     305
7       50% Fwd     312
8       75% Fwd     315
9-10    Full Fwd    325
```

### SYSTEM Subsystem

**Purpose:** Monitor overall system health

**Data Fields:**
- `heartbeat_age`: Float - Seconds since last heartbeat

**Health Check:**
- Normal: heartbeat_age < 1.0 seconds
- Warning: heartbeat_age 1.0-5.0 seconds
- Critical: heartbeat_age > 5.0 seconds

---

## Error Handling

### Serial Communication Errors

The bridge handles several error conditions:

1. **Unicode Decode Errors**: Malformed serial data is skipped
2. **Parse Errors**: Invalid message format is logged and skipped
3. **Serial Port Disconnection**: Fatal error, requires restart

### UDP Broadcast Errors

1. **Socket Errors**: Logged but non-fatal
2. **JSON Encoding Errors**: Logged and skipped

### Statistics Tracking

```python
self.stats = {
    'messages_received': 0,   # Total serial messages
    'messages_parsed': 0,     # Successfully parsed messages
    'broadcasts_sent': 0,     # UDP packets sent
    'errors': 0               # Error count
}
```

**Monitoring Health:**
- `messages_parsed / messages_received` should be > 0.95
- `errors` should grow slowly or not at all

---

## Troubleshooting

### Bridge Won't Start

**Symptom:** `Failed to open serial port: [Errno 2] No such file or directory: '/dev/teensy'`

**Solutions:**
1. Check Teensy is connected: `lsusb | grep Teensy`
2. Check udev rule: `ls -l /dev/teensy`
3. Reload udev rules: `sudo udevadm control --reload-rules && sudo udevadm trigger`

### No UDP Broadcasts Received

**Symptom:** Monitoring client receives no data

**Solutions:**
1. Check firewall: `sudo ufw status` (allow port 6003)
2. Check network interface: `ip addr show`
3. Test with netcat: `nc -ul 6003`
4. Verify bridge is running: `ps aux | grep teensy_serial_bridge`

### High Error Rate

**Symptom:** `errors` count increasing rapidly in statistics

**Possible Causes:**
1. Serial baud rate mismatch (check Teensy code)
2. Noisy USB connection (try different cable/port)
3. Message format mismatch (verify Teensy output format)

**Debug Steps:**
```python
# Add debug logging in parse_message()
logger.debug(f"Raw line: {line}")
logger.debug(f"Parsed: {parsed}")
```

### Data Age Warnings

**Symptom:** `age` values in broadcasts > 1.0 second

**Possible Causes:**
1. Teensy not sending subsystem updates
2. Serial buffer overflow
3. Parse failures preventing data updates

**Check Teensy Code:**
```cpp
// Ensure regular updates for all subsystems
void loop() {
    send_radio_status();   // Should execute every loop
    send_steer_status();
    send_trans_status();
    send_heartbeat();
}
```

---

## Performance Characteristics

### Typical Operation

- **Serial Messages Received:** ~50-100 Hz (from Teensy)
- **Parse Success Rate:** > 95%
- **UDP Broadcasts:** Exactly 5 Hz (rate-limited)
- **CPU Usage:** < 5% on Raspberry Pi 5
- **Memory Usage:** ~15-20 MB

### Network Bandwidth

- **Per Broadcast:** ~300-500 bytes (JSON)
- **Total Rate:** ~1.5-2.5 KB/s (5 Hz × 400 bytes)
- **Very low bandwidth** - suitable for WiFi/Ethernet

---

## Integration Points

### Future Enhancements

The bridge is designed to support bidirectional communication:

```python
# TODO: Add command writing capability
def send_command_to_teensy(self, cmd_type, data):
    """Send navigation commands back to Teensy"""
    # Format: <cmd_type>,<linear_x>,<angular_z>
    command = f"{cmd_type},{data['linear_x']},{data['angular_z']}\n"
    self.ser.write(command.encode())
```

### ROS 2 Integration

The UDP broadcast format makes it easy to create a ROS 2 node:

```python
# Example ROS 2 subscriber
class TeensyStatusSubscriber(Node):
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 6003))
        
        # Create ROS publishers
        self.steering_pub = self.create_publisher(SteeringStatus, 'steering_status', 10)
        
    def listen(self):
        data, _ = self.sock.recvfrom(4096)
        message = json.loads(data.decode())
        # Publish to ROS topics
```

---

## Related Documentation

- **Teensy Firmware:** See `teensy_main.cpp` for message generation
- **udev Rules:** See `udev_rules` for device naming
- **System Architecture:** See main project `README.md`

---

## Changelog

### Version 1.0 (January 2025)
- Initial release
- Serial to UDP bridge functionality
- Support for RADIO, STEER, TRANS, SYSTEM subsystems
- 5 Hz broadcast rate
- Statistics and error tracking

---

## License

Part of the Tractor2025 autonomous platform project.

## Contact

For questions or issues: aej2126 _at_ protonmail !dot! com
