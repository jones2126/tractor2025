# Tractor2025 Project

An autonomous outdoor robot platform for farming and research applications.

## Overview

This repository contains the under development codebase for an outdoor robot platform initially built on an 18HP lawn tractor chassis. The system uses a dual-processor architecture combining low-level hardware control with high-level navigation and perception capabilities.

## System Architecture

### Hardware Platform
- **Main Chassis**: Cub Cadet XT1 LT42 with EFI riding mower tractor-based outdoor robot platform
- **Primary Compute**: Raspberry Pi 5 (for high-level control including navigation)
- **Microcontroller**: Teensy 3.5 (low-level hardware control)
- **Communication**: NRF24 radio link with custom radio controller; RPi wi-fi
- **GPS**: Ardusimple ZED-F9P one for base link and one to calculate heading
- **Base Station**: Custom base station to send RTCM correction data.  See https://github.com/jones2126/tractor2025/blob/main/BridgevilleRTKBase/README.md

### Key Components

#### Teensy 3.5 (Low-Level Control)
- JRK G2 motor controller → Linear actuator → Hydrostatic transmission
- IBT-2 motor controller for steering control with PID feedback
- Steering angle sensor (potentiometer on pin A9/pin 23)
- Wheel encoders (induction sensors) on left and right rear wheels
- NRF24 radio communication with remote control unit
- 4 Unit Relay GPIO board for safety systems

#### Raspberry Pi 5 (High-Level Control)
- Navigation stack (ROS 2 + Pure Pursuit algorithm)
- Dual ZED-F9P RTK GNSS receivers for precision localization and heading
- OAK camera for AprilTag localization and teleoperation
- Internet connectivity for RTK correction data
- Interface to Teensy 3.5 for hardware control

#### Localization System
- **Primary**: Dual on-board RTK GNSS receivers
- **Base Station**: Private RTK base station with Wi-Fi communication
- **Secondary**: Wheel odometry from rear wheel sensors
- **Vision**: OAK camera to be used for teleoperation in conjunction with wi-fi control

#### Remote Control
- Custom-built radio controller with:
  - 1 Teensy 3.2
  - 4 potentiometers (steering, throttle, aux controls)
  - 3 pushbuttons
  - 1 toggle switch
  - Emergency stop functionality
  - 1 NRF24 radio

## Control Systems

### Steering Control
- **Implementation**: PID-based steering control with potentiometer used for steer angle data
- **Hardware**: IBT-2 motor controller driving steering motor
- **Feedback**: Potentiometer on pin A9 (pin 23) provides steering angle
- **Modes**: Manual (radio control) and Auto (ROS cmd_vel integration planned)

### Transmission Control
- **Implementation**: Software defined 10-bucket speed control approach for speed transitions to mimic gear settings
- **Hardware**: JRK G2 motor controller for linear actuator control
- **Range**: Full reverse to full forward with neutral position on hydrostatic transmission which requires ~3" travel

### Safety Systems
- **E-Stop**: Radio-controlled emergency stop with relay able to ground ignition wire to ground
- **Signal Loss Protection**: Automatic safe state when radio communication is lost either to radio control or wi-fi
- **Manual Override**: Physical e-stop button on tractor

## Teensy 3.5 Hardware Pin Assignments 

### Analog Inputs
- **A9 (Pin 23)**: Steering angle potentiometer ✅
- **Pin 24**: ⚠️ **AVOID** - Not analog-capable despite some documentation

### Digital I/O
- **Pin 5**: IBT-2 RPWM (steering motor control)
- **Pin 6**: IBT-2 LPWM (steering motor control)
- **Pin 32**: E-stop relay control
- **Pins 9, 10**: NRF24 radio (CE, CSN)
- **Pins 27, 28, 39**: NRF24 radio SPI pins (SCK, MOSI, MISO)

### Serial Communications
- **Serial3**: JRK G2 motor controller communication

## Radio Control Operating Modes

- **Manual**: Direct radio control operation with PID steering assistance
- **Automated**: Autonomous navigation with Pure Pursuit (in development)
- **Paused/Standby**: System idle state

## Development Environment

### Software Stack
- **Languages**: C++ (Teensy), Python (RPi)
- **Frameworks**: RPi: Ubuntu 24.04, ROS 2, PlatformIO
- **Navigation**: Pure Pursuit algorithm
- **Remote Access**: NoMachine, FileZilla, PuTTY, ZeroTier for VPN

### Development Workflow
1. Code editing on laptop using VSCode
2. Commit changes to GitHub repository
3. From Raspberry Pi 5 on tractor pull updates from GitHub 
4. Compile and upload Teensy code using PlatformIO CLI
5. Deploy and test on tractor robot platform

## Repository Structure

```
NOT ACCURATE; JUST A PLACEHOLDER
tractor2025/
├── teensy/                 # Teensy 3.5 firmware (C++)
│   ├── src/               # Source code
│   ├── lib/               # Libraries
│   └── platformio.ini     # PlatformIO configuration
├── rpi/                   # Raspberry Pi code (Python)
│   ├── src/               # Source code
│   ├── launch/            # ROS 2 launch files
│   └── config/            # Configuration files
├── docs/                  # Documentation
└── README.md             # This file
```

## Getting Started

### Prerequisites - Robot
- Raspberry Pi 5 with Ubuntu 24 and ROS 2 installed
- PlatformIO CLI for Teensy development
- RTK base station configured and operational
- GPS units, JRK G2, IBT-2, 4 port relay
- Teensy 3.5

### Prerequisites - Radio Control Unit
- Teensy 3.2
- Custom designed PCB
- Multiple components soldered to PCB (e.g. potentiometers, RGB LEDs, switch, pushbutton, etc.)

### Getting Started
1. Join the Slack Group:
   ```bash
     Sign up for Slack access to Lawn Tractor Automation: https://app.slack.com/client/T8WP3RHH7/C8YBK20LX
   ```

2. Engage with small group:
   ```bash
    Join Zoom meeting on Thursdays at Noon (ET)) https://us02web.zoom.us/j/82088036016?pwd=K2lLc1FiWm9MU0dzRStxM2J2b3dpQT09#success
   ```

3. Review previous meetings on YouTube:
   ```bash
    Lawn Tractor Automation YouTube page:  https://www.youtube.com/@lawntractorautomation2726/videos
   ```

4. Review details on shared Miro Board:
   ```bash
    Miro board: https://miro.com/app/board/uXjVM1yzdFo=/
   ```

## Hardware Notes & Troubleshooting

### Teensy 3.5 Pin Considerations
- **Pin 24 Issue**: pin 24 is NOT analog-capable. Always refer to the official pinout diagram.


## Safety Features
- Emergency stop via radio controller with relay-based ignition kill
- Manual e-stop on tractor
- Automatic safe state on radio signal loss
- PID-controlled steering with deadband for stability

## Contact
[aej2126 _at_ protonmail !dot! com]