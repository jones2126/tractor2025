# Teensy Serial Message Format - Quick Reference

## Message Structure
```
<msg_type>,<timestamp>,<subsystem>,<data>
```

## Message Type 1: Status/Logging (for RPi)
**Purpose:** Operational data that RPi should monitor and log

### Format Examples

#### Simple Status
```
1,12345,SYSTEM,heartbeat
1,12456,RADIO,initialized_success
```

#### Single Key-Value
```
1,15678,JRK,target=2048.00
```

#### Multiple Key-Values
```
1,18901,RADIO,ack_rate=9.8,current_rate=10.1,total_acks=98,signal=GOOD,struct_size=32
1,21234,STEER,mode=1,setpt=512,current=510,error=2,dir=LEFT,pwm=45
1,23567,TRANS,mode=1,bucket=5,target=2985,current=2985
```

### Python Parsing Example
```python
def parse_type1_message(line):
    """Parse Type 1 status/logging message"""
    parts = line.strip().split(',', 3)  # Split into max 4 parts
    
    if len(parts) < 3:
        return None
    
    msg_type = int(parts[0])
    timestamp = int(parts[1])
    subsystem = parts[2]
    
    # Parse data section (may have multiple key=value pairs)
    data = {}
    if len(parts) > 3:
        data_str = parts[3]
        
        # Check if it's simple text or key=value pairs
        if '=' in data_str:
            # Parse key=value pairs
            pairs = data_str.split(',')
            for pair in pairs:
                if '=' in pair:
                    key, value = pair.split('=', 1)
                    # Try to convert to number, otherwise keep as string
                    try:
                        data[key] = float(value)
                    except ValueError:
                        data[key] = value
        else:
            # Simple text message
            data['message'] = data_str
    
    return {
        'type': msg_type,
        'timestamp': timestamp,
        'subsystem': subsystem,
        'data': data
    }

# Example usage:
msg = "1,18901,STEER,mode=1,setpt=512,current=510,error=2,dir=LEFT,pwm=45"
parsed = parse_type1_message(msg)
# Result:
# {
#     'type': 1,
#     'timestamp': 18901,
#     'subsystem': 'STEER',
#     'data': {
#         'mode': 1.0,
#         'setpt': 512.0,
#         'current': 510.0,
#         'error': 2.0,
#         'dir': 'LEFT',
#         'pwm': 45.0
#     }
# }
```

## Message Type 2: Debug (filtered in production)
**Purpose:** Development and troubleshooting

### Format Examples
```
2,12345,SYSTEM,debug_msg_0
2,12456,STEER_POT,raw=512.00
2,12567,STEER_POT,voltage=1.65
```

### Python Parsing (same as Type 1)
```python
# Same parsing function works for Type 2
parsed = parse_type1_message("2,12456,STEER_POT,raw=512.00")
# Result:
# {
#     'type': 2,
#     'timestamp': 12456,
#     'subsystem': 'STEER_POT',
#     'data': {'raw': 512.0}
# }
```

## Subsystem Reference

| Subsystem | Messages | Update Rate | Description |
|-----------|----------|-------------|-------------|
| SYSTEM | heartbeat, startup msgs | 5s / once | General system status |
| RADIO | ack rates, signal quality | 10s / 5s | Radio communication |
| JRK | target position | 5s | Transmission motor controller |
| TRANS | mode, bucket, target | 5s | Transmission control state |
| TRANS_LOG | detailed CSV data | 10 Hz | Detailed transmission logging |
| STEER | mode, setpoint, error | 2s | Steering control state |
| STEER_POT | raw pot readings | Debug only | Potentiometer diagnostics |

## Expected Message Rates

### Type 1 (Status/Logging)
- **SYSTEM heartbeat**: Every 5 seconds
- **RADIO stats**: Every 10 seconds  
- **RADIO data**: Every 5 seconds
- **JRK target**: Every 5 seconds (when changed)
- **TRANS status**: Every 5 seconds
- **TRANS_LOG**: 10 Hz (every 100ms)
- **STEER status**: Every 2 seconds

### Type 2 (Debug)
- Only enabled when debugging
- Typically 1 Hz when active

## Filtering Messages on RPi

### Option 1: Filter by Type
```python
def should_log_message(msg_type):
    """Only log Type 1 messages in production"""
    return msg_type == 1
```

### Option 2: Filter by Subsystem
```python
IMPORTANT_SUBSYSTEMS = ['RADIO', 'TRANS', 'STEER', 'JRK']

def should_log_message(subsystem):
    """Only log important subsystems"""
    return subsystem in IMPORTANT_SUBSYSTEMS
```

### Option 3: Filter by Update Rate
```python
from collections import defaultdict
import time

last_logged = defaultdict(float)
MIN_INTERVAL = {
    'SYSTEM': 5.0,      # Max 1 msg per 5 seconds
    'RADIO': 5.0,       # Max 1 msg per 5 seconds
    'STEER': 2.0,       # Max 1 msg per 2 seconds
    'TRANS': 5.0,       # Max 1 msg per 5 seconds
    'TRANS_LOG': 0.1,   # Max 10 Hz
}

def should_log_message(subsystem):
    """Rate-limit logging per subsystem"""
    now = time.time()
    interval = MIN_INTERVAL.get(subsystem, 1.0)
    
    if now - last_logged[subsystem] >= interval:
        last_logged[subsystem] = now
        return True
    return False
```

## Complete RPi Serial Handler Example

```python
import serial
import time
from collections import defaultdict

class TeensySerialHandler:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.last_messages = defaultdict(dict)
        
    def parse_message(self, line):
        """Parse incoming message"""
        parts = line.strip().split(',', 3)
        
        if len(parts) < 3:
            return None
            
        try:
            msg_type = int(parts[0])
            timestamp = int(parts[1])
            subsystem = parts[2]
            
            data = {}
            if len(parts) > 3:
                data_str = parts[3]
                if '=' in data_str:
                    pairs = data_str.split(',')
                    for pair in pairs:
                        if '=' in pair:
                            key, value = pair.split('=', 1)
                            try:
                                data[key] = float(value)
                            except ValueError:
                                data[key] = value
                else:
                    data['message'] = data_str
            
            return {
                'type': msg_type,
                'timestamp': timestamp,
                'subsystem': subsystem,
                'data': data
            }
        except (ValueError, IndexError) as e:
            print(f"Parse error: {e} for line: {line}")
            return None
    
    def handle_message(self, parsed):
        """Process parsed message"""
        if parsed['type'] == 1:  # Status/logging
            # Store latest status for each subsystem
            self.last_messages[parsed['subsystem']] = parsed
            
            # Take action based on subsystem
            if parsed['subsystem'] == 'RADIO':
                if 'signal' in parsed['data'] and parsed['data']['signal'] == 'LOST':
                    print("WARNING: Radio signal lost!")
                    
            elif parsed['subsystem'] == 'STEER':
                # Monitor steering for anomalies
                if 'error' in parsed['data'] and abs(parsed['data']['error']) > 100:
                    print(f"WARNING: Large steering error: {parsed['data']['error']}")
                    
        elif parsed['type'] == 2:  # Debug
            # Only print debug in development mode
            if self.debug_mode:
                print(f"DEBUG: {parsed['subsystem']}: {parsed['data']}")
    
    def run(self):
        """Main loop"""
        print("Starting Teensy serial handler...")
        while True:
            if self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8')
                    parsed = self.parse_message(line)
                    if parsed:
                        self.handle_message(parsed)
                except UnicodeDecodeError:
                    pass  # Skip malformed messages
            time.sleep(0.01)  # 100 Hz check rate

# Usage:
if __name__ == "__main__":
    handler = TeensySerialHandler('/dev/ttyACM0')
    handler.debug_mode = False  # Set to True for debug messages
    handler.run()
```

## Testing Messages

You can test parsing with these example messages:

```python
test_messages = [
    "1,12345,SYSTEM,heartbeat",
    "1,15678,RADIO,ack_rate=9.8,current_rate=10.1,signal=GOOD",
    "1,18901,STEER,mode=1,setpt=512,current=510,error=2,dir=LEFT,pwm=45",
    "1,21234,TRANS,mode=1,bucket=5,target=2985,current=2985",
    "2,12456,STEER_POT,raw=512.00",
]

for msg in test_messages:
    parsed = parse_type1_message(msg)
    print(f"Input:  {msg}")
    print(f"Output: {parsed}\n")
```

## Next: Sending Commands to Teensy

After verifying you can receive and parse Teensy messages, we'll add:

1. **Type 1 messages FROM RPi** (cmd_vel data at 20 Hz)
2. **Type 2 messages FROM RPi** (preset updates)

Format will be similar but reversed direction.
