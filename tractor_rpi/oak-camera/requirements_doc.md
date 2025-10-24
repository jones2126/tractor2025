# Robot Teleoperation System Requirements

## Current Implementation Status

### ✅ Completed Features

#### Video System
- Live video streaming from Oak Camera (DepthAI 3.0.0)
- Real-time overlay system with:
  - Crosshair for targeting
  - Grid overlay for spatial reference
  - Telemetry display in upper-left corner
  - Timestamp display
- Configurable overlay settings (on/off toggles)

#### Control Interface
- **Steering Control**: 7-position steering system
  - Full Left (-1.0) → 1/2 Left (-0.5) → 1/4 Left (-0.25) → Straight (0.0) → 1/4 Right (0.25) → 1/2 Right (0.5) → Full Right (1.0)
  - Visual feedback showing current steering position
  - Maps directly to ROS2 cmd_vel angular values

- **Speed Control**: 10-position transmission system
  - 6 Forward speeds (Forward 1-6)
  - 1 Neutral/Stop position (prominent styling)
  - 3 Reverse speeds (Reverse 1-3)
  - Radio button interface for discrete selection
  - Maps to linear actuator positions

- **Emergency Stop**: Red button in top-right corner
  - Immediately sets speed to neutral and steering to straight
  - Sends zero velocity command to robot

#### Communication
- WebSocket-based real-time control
- Latency monitoring and display
- ROS2 cmd_vel command generation
- Status update broadcasting

#### User Interface
- Responsive web interface accessible at http://192.168.1.151:5000
- Clean layout with video on left, controls on right
- Telemetry overlay shows: Battery, Connection, Latency, RTK Fix, Autopilot status

### 🔧 Integration Points (TODO)

#### ROS2 Integration
```python
# TODO: Implement in handle_cmd_vel()
twist_msg = Twist()
twist_msg.linear.x = linear    # -1.0 to 1.0 from speed control
twist_msg.angular.z = angular  # -1.0 to 1.0 from steering control
cmd_vel_publisher.publish(twist_msg)
```

#### Hardware Status Monitoring
```python
# TODO: Implement in status_update_thread()
robot_status['battery'] = get_battery_level()      # From battery monitor
robot_status['rtk_fix'] = get_rtk_status()         # From GPS/RTK system
robot_status['autopilot'] = get_autopilot_status() # From navigation system
```

## Future Development Requirements

### Phase 2: GPS Mission Planning Interface

#### Requirements
- **Dual-screen support**: Teleoperation + Mission Planning
- **2D GPS mapping interface** showing:
  - Planned mission waypoints
  - Real-time GPS track (breadcrumb trail)
  - Current robot position
  - RTK fix quality indicator
  - Planned vs actual path deviation

#### Technical Specifications
- Integration with mapping library (Leaflet.js or similar)
- Real-time GPS coordinate streaming
- Mission waypoint upload/download
- Geographic coordinate system support (WGS84)

### Phase 3: Autonomous Operation Integration

#### RTK Fix Monitoring
- **Continuous RTK status monitoring**
- **Automatic failover logic**:
  - RTK Fix Lost → Autopilot Disengages → Robot Stops → Teleoperation Required
  - RTK Fix Restored → Option to Re-engage Autopilot

#### Autopilot Integration
- Start/Stop autopilot from teleoperation interface
- Mission progress monitoring
- Real-time status updates during autonomous operation

### Phase 4: Enhanced Telemetry

#### Additional Sensor Data
- Motor current/temperature
- Hydraulic pressure
- Implement temperature
- GPS accuracy (CEP values)
- Satellite count
- Speed over ground vs commanded speed

#### Data Logging
- Mission data recording
- Telemetry logging for post-mission analysis
- Video recording capability

## Technical Architecture

### Current Stack
- **Backend**: Python Flask + SocketIO
- **Frontend**: HTML5 + JavaScript + WebSocket
- **Video**: DepthAI 3.0.0 with Oak Camera
- **Communication**: WebSocket for real-time control

### Required Dependencies
```bash
pip install flask flask-socketio opencv-python depthai numpy
```

### Hardware Requirements
- Raspberry Pi 5
- Oak Camera (with DepthAI 3.0.0 support)
- USB connection between Pi and Oak Camera
- Network connection for web interface access

### Future Integration Points
- **ROS2**: For robot control and sensor data
- **GPS/RTK**: For positioning and navigation
- **CAN Bus**: For hydraulic/motor control
- **Additional cameras**: For 360° situational awareness

## Operational Workflow

### Current Teleoperation Mode
1. Operator accesses web interface
2. Verifies video feed and telemetry
3. Uses steering buttons to set direction
4. Uses speed radio buttons to set transmission position
5. Robot responds to cmd_vel commands
6. Emergency stop available at all times

### Future Autonomous Mode Workflow
1. **Pre-Mission**: Load GPS waypoints via mission planning interface
2. **Manual Start**: Use teleoperation to position robot at starting point
3. **Engage Autopilot**: Switch to autonomous navigation
4. **Monitor Progress**: Watch mission progress on GPS interface
5. **RTK Failover**: If RTK lost, automatic stop and teleoperation takeover
6. **Mission Complete**: Return to teleoperation for final positioning

## Configuration Notes

### Network Setup
- Pi IP: 192.168.1.151
- Web interface port: 5000
- Ensure firewall allows incoming connections on port 5000

### Camera Configuration
- Resolution: 1080p
- Frame rate: 30 FPS
- Color format: BGR
- Automatic reconnection on device errors

### Control Mapping
- **Steering**: -1.0 (full left) to +1.0 (full right)
- **Speed**: Position 3-12 mapped to linear velocity -1.0 to +1.0
- **Neutral**: Position 6 = 0.0 velocity (stop)

## Safety Considerations

### Emergency Procedures
- Emergency stop button prominently placed
- Automatic stop on RTK fix loss during autopilot
- Manual override always available
- Connection loss should trigger robot stop

### Operational Limits
- Teleoperation required for:
  - Initial positioning
  - RTK fix loss recovery
  - Manual override situations
  - Precise maneuvering

### Future Safety Enhancements
- Obstacle detection integration
- Geofencing for operational boundaries
- Operator deadman switch
- Automatic timeout if no operator input
