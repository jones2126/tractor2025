# Robot Status LED Tower Documentation

## Overview
This system uses a 4-LED signal tower to provide real-time visual feedback about the robot's operational status. Each LED color represents a different aspect of system health and operation.

## Hardware Setup

### Components
- 4x 1W RGB LED modules
- PCA9685 16-channel PWM servo driver
- LED drivers (one per LED)
- Power supply (appropriate for your LEDs)

### Wiring
```
PCA9685 Channel Assignments:
- Channel 8:  Red LED (standalone)
- Channel 9:  Green LED (standalone)
- Channel 10: Blue LED (standalone)
- Channel 11: Yellow LED - Green component
- Channel 12: Yellow LED - Red component

Connect PCA9685 to Raspberry Pi I2C:
- SDA → GPIO 2 (Pin 3)
- SCL → GPIO 3 (Pin 5)
- VCC → 3.3V or 5V (depending on board)
- GND → Ground
```

## LED Status Indicators

### 🟡 YELLOW LED - GPS/RTCM/RTK Status

| Pattern | Meaning | System State |
|---------|---------|--------------|
| Slow fade (0.5Hz) | Boot sequence | System initializing, no GPS lock |
| 3Hz blink | GPS active | GPS fix acquired, no RTCM corrections |
| Solid ON | RTCM active | Receiving RTK correction data |
| Fast blink (5Hz) | RTK Fixed | ⭐ Centimeter-level accuracy achieved |
| Alternating bright/dim | RTK Lost | ⚠️ Warning - RTK fix lost, degraded accuracy |

### 🔵 BLUE LED - Control Mode & Communication

| Pattern | Meaning | System State |
|---------|---------|--------------|
| OFF | No communication | ⚠️ No radio or WiFi connection |
| 1Hz slow blink | NRF24 Manual | Manual control via radio controller |
| 2Hz medium blink | WiFi Manual | Manual control via WiFi interface |
| Solid ON | Auto + RTK Ready | ⭐ Ready for autonomous operation |
| 5Hz fast blink | Auto + RTK Lost | ⚠️ PAUSED - Requires manual intervention |
| Alternating pattern | Mission executing | Autonomous mission in progress |

### 🟢 GREEN LED - System Health

| Pattern | Meaning | System State |
|---------|---------|--------------|
| OFF | System off | Not yet initialized |
| 0.5Hz slow blink | Starting up | Boot sequence in progress |
| Solid ON | All systems nominal | ⭐ Everything operating normally |
| 2Hz medium blink | Minor warning | ⚠️ Non-critical issue (e.g., low battery) |
| 5Hz fast blink | Critical warning | ⚠️ Immediate attention needed |

### 🔴 RED LED - Safety & Errors

| Pattern | Meaning | System State |
|---------|---------|--------------|
| OFF | No errors | System safe, no issues |
| 1Hz slow blink | E-stop armed | E-stop circuit ready but not triggered |
| 5Hz fast blink | Safety violation | ⚠️ Communication timeout or safety issue |
| Solid ON | E-STOP ACTIVE | 🛑 Emergency stop triggered |
| Alternating pattern | Recoverable error | Error state, attempting recovery |

## Installation

### 1. Install Dependencies
```bash
# Install Python packages
pip3 install adafruit-circuitpython-pca9685
pip3 install adafruit-blinka

# Ensure I2C is enabled
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable
```

### 2. Install the LED Controller
```bash
# Copy the script to your ROS 2 workspace
cp robot_status_leds.py ~/ros2_ws/src/

# Make it executable
chmod +x ~/ros2_ws/src/robot_status_leds.py

# Copy the systemd service file
sudo cp robot_status_leds.service /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable service to start at boot
sudo systemctl enable robot_status_leds.service

# Start the service now
sudo systemctl start robot_status_leds.service
```

### 3. Verify Installation
```bash
# Check service status
sudo systemctl status robot_status_leds.service

# View logs
sudo journalctl -u robot_status_leds.service -f
```

## ROS 2 Topics

The LED controller subscribes to these topics:

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/fix` | `sensor_msgs/NavSatFix` | GPS position and RTK status |
| `/rtcm_status` | `std_msgs/Bool` | RTCM correction data reception |
| `/control_mode` | `std_msgs/String` | "manual" or "auto" |
| `/nrf_status` | `std_msgs/Bool` | NRF24 radio connection status |
| `/estop` | `std_msgs/Bool` | Emergency stop state |

### Creating Missing Topics

You'll need to publish to these topics from your other nodes. Examples:

```python
# Publishing RTCM status
import rclpy
from std_msgs.msg import Bool

rtcm_pub = node.create_publisher(Bool, '/rtcm_status', 10)
msg = Bool()
msg.data = True  # RTCM data being received
rtcm_pub.publish(msg)

# Publishing control mode
from std_msgs.msg import String

mode_pub = node.create_publisher(String, '/control_mode', 10)
msg = String()
msg.data = "auto"  # or "manual"
mode_pub.publish(msg)
```

## Tuning LED Patterns

### Adjusting Brightness
Edit the pattern brightness values in `robot_status_leds.py`:

```python
# Example: Make yellow LED brighter during boot fade
self.yellow_pattern.set_pattern("fade", frequency=0.5, brightness=100)  # was 80
```

### Adjusting Blink Rates
```python
# Example: Make GPS blink slower
self.yellow_pattern.set_pattern("blink", frequency=2.0, brightness=100)  # was 3.0
```

### Adjusting Yellow Color Balance
If yellow looks too orange or too green:

```python
# Near the top of the file, adjust these values:
YELLOW_GAIN_R = 1.00  # Red component (0.0 to 1.0)
YELLOW_GAIN_G = 0.90  # Green component (0.0 to 1.0)

# More green: increase YELLOW_GAIN_G
# More orange: decrease YELLOW_GAIN_G
```

### Adjusting PWM Frequency
If LEDs flicker or are too dim:

```python
# Near the top of the file:
FREQ_HZ = 500  # Try values between 300-800 Hz
```

## Troubleshooting

### LEDs Not Working

**Check I2C connection:**
```bash
# Should show device at 0x40 (default PCA9685 address)
sudo i2cdetect -y 1
```

**Check service status:**
```bash
sudo systemctl status robot_status_leds.service
```

**Check for errors:**
```bash
sudo journalctl -u robot_status_leds.service -n 50
```

### LEDs Too Dim or Too Bright

1. Check PWM frequency (try different values 300-800 Hz)
2. Adjust pattern brightness values (0-100)
3. Check power supply voltage to LEDs
4. Verify LED driver current settings

### Inverted Logic (LED Bright When Should Be Dark)

Set the appropriate inversion flag to True:

```python
INVERT_RED = True      # If red LED is inverted
INVERT_GREEN = True    # If green LED is inverted
INVERT_BLUE = True     # If blue LED is inverted
INVERT_YELLOW_R = True # If yellow red component is inverted
INVERT_YELLOW_G = True # If yellow green component is inverted
```

### ROS Topics Not Publishing

Verify topics exist:
```bash
ros2 topic list
```

Echo a topic to see if data is being published:
```bash
ros2 topic echo /fix
```

### Service Won't Start at Boot

Check dependencies:
```bash
# Make sure ROS 2 is sourced in service file
sudo systemctl edit robot_status_leds.service
```

Verify user permissions:
```bash
# Make sure 'tractor' user has I2C access
sudo usermod -a -G i2c tractor
```

## Testing the System

### Manual Testing

```bash
# Test individual topics
ros2 topic pub /rtcm_status std_msgs/Bool "data: true"
ros2 topic pub /control_mode std_msgs/String "data: 'auto'"
ros2 topic pub /nrf_status std_msgs/Bool "data: true"
```

### View Current Status
```bash
# Watch the logs in real-time
sudo journalctl -u robot_status_leds.service -f
```

## Maintenance

### Regular Checks
- Verify LED brightness consistency
- Check for loose connections
- Monitor system logs for errors
- Test all communication pathways

### Log Rotation
Logs are managed by systemd journal. To limit log size:
```bash
# Edit journald configuration
sudo nano /etc/systemd/journald.conf

# Set limits:
SystemMaxUse=100M
SystemMaxFileSize=10M
```

## Future Enhancements

Potential additions to consider:

1. **Battery status indication** - Use green LED patterns for battery levels
2. **Mission progress** - Use blue LED to show waypoint completion percentage
3. **Obstacle detection** - Flash red when obstacles detected
4. **Remote monitoring** - Publish LED status to dashboard
5. **Audio feedback** - Add buzzer for critical alerts
6. **Custom patterns** - User-defined patterns via ROS parameters

## Safety Notes

⚠️ **Important Safety Considerations:**

1. **E-Stop Priority**: Red LED should always be immediately responsive to e-stop signals
2. **Fail-Safe**: If LED controller crashes, ensure robot enters safe state
3. **Redundant Monitoring**: Don't rely solely on LEDs for safety - use audible alarms for critical issues
4. **Visibility**: Ensure LEDs are visible from all operator positions
5. **Power**: Use separate power supply for LEDs to prevent robot brownout

## Support

For issues or questions:
- GitHub Issues: https://github.com/jones2126/tractor2025/issues
- Slack: Lawn Tractor Automation group
- Email: aej2126@protonmail.com

## License

This code is part of the tractor2025 project and follows the same license.

---
Last Updated: January 2025
Version: 1.0
