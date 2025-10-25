# Teensy Tractor Main Firmware Documentation

## Overview

The Teensy 3.5 firmware (`tractor_teensy_main.cpp`) is the low-level hardware control system for the autonomous tractor platform. It manages steering, transmission, radio communication, and safety systems, bridging between radio control/autonomous commands and physical hardware actuators.

**Primary Functions:**
1. Receives commands via NRF24 radio from remote control unit
2. Controls steering via IBT-2 motor controller with PID feedback
3. Controls transmission via JRK G2 motor controller with 10-bucket speed system
4. Monitors e-stop relay for safety shutdown
5. Publishes status via serial to Raspberry Pi bridge

**Hardware:** Teensy 3.5  
**Version:** 1.0  
**Author:** Tractor2025 Project  
**Last Updated:** January 2025

---

## System Architecture

```
┌──────────────────┐
│  Radio Control   │  NRF24 Radio
│  (Teensy 3.2)    │──────────────┐
└──────────────────┘              │
                                  │ 915MHz
                                  │ 250kbps
                                  ▼
                       ┌─────────────────────┐
                       │  Teensy 3.5         │
                       │  Main Controller    │
                       └──────────┬──────────┘
                                  │
                ┌─────────────────┼─────────────────┐
                ▼                 ▼                 ▼
        ┌──────────────┐  ┌──────────────┐  ┌──────────────┐
        │ IBT-2 Motor  │  │ JRK G2       │  │ E-Stop Relay │
        │ Controller   │  │ Controller   │  │ (Pin 32)     │
        └──────┬───────┘  └──────┬───────┘  └──────────────┘
               │                 │
               ▼                 ▼
        ┌──────────────┐  ┌──────────────┐
        │ Steering     │  │ Linear       │
        │ Motor        │  │ Actuator     │
        └──────────────┘  └──────────────┘
               ▲                 
               │                 
        ┌──────────────┐         
        │ Steering Pot │         
        │ (Pin A9/23)  │         
        └──────────────┘         

                       Serial USB
                           │
                           ▼
                  ┌─────────────────┐
                  │ Raspberry Pi 5  │
                  │ Serial Bridge   │
                  └─────────────────┘
```

---

## Pin Assignments

### Teensy 3.5 Pinout

| Pin | Function | Connection | Notes |
|-----|----------|------------|-------|
| **A9 (23)** | Steering Pot | Analog input | 0-1023 range |
| **5** | IBT-2 RPWM | PWM output | Right motor direction |
| **6** | IBT-2 LPWM | PWM output | Left motor direction |
| **32** | E-Stop Relay | Digital output | HIGH=safe, LOW=stop |
| **9** | NRF24 CE | Digital output | Radio chip enable |
| **10** | NRF24 CSN | Digital output | Radio chip select |
| **11** | SPI MOSI | SPI | Radio data out |
| **12** | SPI MISO | SPI | Radio data in |
| **13** | SPI SCK | SPI | Radio clock |
| **Serial3** | JRK G2 TX/RX | UART | 9600 baud |

### Critical Pin Notes

⚠️ **Pin A9 (Physical Pin 23)**: This is the ONLY analog-capable pin being used for the steering potentiometer. Do not use pin 24 as it is not analog-capable despite some documentation.

---

## Radio Communication (NRF24)

### Radio Configuration

```cpp
// RF24 radio(CE_PIN, CSN_PIN)
RF24 radio(9, 10);

// Settings
radio.setPALevel(RF24_PA_HIGH);      // High power
radio.setDataRate(RF24_250KBPS);     // 250 kbps for reliability
radio.setChannel(124);                // Channel 124
```

### Addresses

```cpp
const uint8_t address[][6] = {"RCTRL", "TRACT"};
// "RCTRL" = Radio Control (send ACKs back)
// "TRACT" = Tractor (listen for incoming data)
```

### Data Structures

**Received from Radio Control:**
```cpp
struct RadioControlStruct {
    float steering_val;      // Steering position (0-1023)
    float throttle_val;      // Throttle position (0-1023)
    float voltage;           // Control unit battery voltage
    float transmission_val;  // Transmission control (0-1023)
    float pot4_val;          // Auxiliary pot (future use)
    byte estop;              // Emergency stop (0=safe, 1=stop)
    byte control_mode;       // 0=Pause, 1=Manual, 2=Auto
    byte button01;           // Button state
    byte button02;           // Button state
};
```

**Acknowledgment Sent Back:**
```cpp
struct AckPayloadStruct {
    unsigned long counter;   // Increments with each ACK
    uint32_t dummy[4];       // Padding/debug data
};
```

### Radio Statistics Tracking

```cpp
struct RadioStats {
    unsigned long lastAckTime;           // Last successful reception
    unsigned long ackCount;              // Total ACKs sent
    unsigned long shortTermAckCount;     // Recent ACK count
    unsigned long lastRateReport;        // Last comprehensive report
    unsigned long lastDataPrint;         // Last data print
    bool signalGood;                     // Signal quality flag
    
    // Timing intervals
    const unsigned long signalTimeout = 2000;       // 2 sec timeout
    const unsigned long rateReportInterval = 10000; // 10 sec reports
    const unsigned long dataPrintInterval = 5000;   // 5 sec data print
};
```

**Signal Quality:**
- **GOOD**: Received data within last 2 seconds
- **LOST**: No data for 2+ seconds → automatic safety mode

### Radio Message Flow

```
Radio Control Unit                    Teensy 3.5
        │                                   │
        │──── RadioControlStruct ────────>│
        │                                   │
        │                         [Process Data]
        │                                   │
        │<──── AckPayloadStruct ───────────│
        │                                   │
```

**Radio Data Print Example (every 5 seconds):**
```
1,45690,RADIO,steer=512.00,throttle=500.00,trans=469.00,volt=12.4,estop=0,mode=1
```

**Radio Rate Report Example (every 10 seconds):**
```
1,56234,RADIO,ack_rate=4.8,current_rate=4.8,total_acks=48,signal=GOOD,struct_size=28
```

---

## Transmission Control System

### 10-Bucket Speed System

The transmission uses a **10-bucket discrete speed system** that maps radio control input to predefined JRK servo positions for smooth, predictable speed control.

```cpp
const uint16_t bucketTargets[10] = {
    3696,   // Bucket 0: Full Reverse (CCW)
    3554,   // Bucket 1: 75% Reverse
    3412,   // Bucket 2: 50% Reverse
    3270,   // Bucket 3: 25% Reverse
    3128,   // Bucket 4: Near Neutral
    2985,   // Bucket 5: Neutral (CENTER)
    2751,   // Bucket 6: 25% Forward
    2517,   // Bucket 7: 50% Forward
    2283,   // Bucket 8: 75% Forward
    2048    // Bucket 9: Full Forward (CW)
};
```

### Bucket Selection Logic (Manual Mode)

Radio control `transmission_val` (0-1023) is mapped to buckets:

```cpp
if (transmission_val >= 931)      bucket = 0;  // Full reverse
else if (transmission_val >= 838) bucket = 1;
else if (transmission_val >= 746) bucket = 2;
else if (transmission_val >= 654) bucket = 3;
else if (transmission_val >= 562) bucket = 4;
else if (transmission_val >= 469) bucket = 5;  // Neutral
else if (transmission_val >= 377) bucket = 6;
else if (transmission_val >= 285) bucket = 7;
else if (transmission_val >= 192) bucket = 8;
else                              bucket = 9;  // Full forward
```

### Visual Bucket Map

```
Reverse Direction ◄────────────────────────────────► Forward Direction
  
  Bucket:  0     1     2     3     4     5     6     7     8     9
           │     │     │     │     │     │     │     │     │     │
  Servo:  3696  3554  3412  3270  3128  2985  2751  2517  2283  2048
           │                             │                           │
       Full Rev                      Neutral                    Full Fwd
       
Radio Val: 1023──────────────────────►469◄──────────────────────────1
```

### JRK G2 Controller Interface

**Communication:** Serial3 @ 9600 baud  
**Protocol:** Pololu JRK G2 compact protocol

**Set Target Command:**
```cpp
void setJrkTarget(uint16_t target) {
    if (target > 4095) target = 4095;
    Serial3.write(0xC0 + (target & 0x1F));
    Serial3.write((target >> 5) & 0x7F);
}
```

**Read Feedback:**
```cpp
uint16_t readFeedback() {
    Serial3.write(0xE5);  // Get variables command
    Serial3.write(0x04);  // Offset for Feedback
    Serial3.write(0x02);  // Length = 2 bytes
    
    // Read response with 100ms timeout
    uint8_t low = Serial3.read();
    uint8_t high = Serial3.read();
    return (high << 8) | low;
}
```

### Transmission Control Loop

**Rate:** 10 Hz (every 100ms)

```cpp
void controlTransmission() {
    // Safety check
    if (!radioStats.signalGood) {
        radioData.control_mode = 9;  // Override to safety mode
    }
    
    uint16_t requestedTarget;
    switch (radioData.control_mode) {
        case 0:  // Pause - go to neutral
            requestedTarget = transmissionNeutralPos;
            break;
            
        case 1:  // Manual - use bucket system
            bucket = calculateBucketFromRadio();
            requestedTarget = bucketTargets[bucket];
            break;
            
        case 2:  // Auto - placeholder for cmd_vel
            requestedTarget = transmissionNeutralPos;
            break;
            
        default:  // Error - go to neutral
            requestedTarget = transmissionNeutralPos;
            break;
    }
    
    setJrkTarget(requestedTarget);
}
```

**Transmission Status Print (every 5 seconds):**
```
1,45702,TRANS,mode=1,bucket=7,target=2517,current=2520
```

**Transmission Log (every 100ms):**
```
1,45702,TRANS_LOG,output=2520,trans_val=285.50,target=2517,bucket=7,feedback=2518
```

---

## Steering Control System

### Hardware Setup

**Motor Controller:** IBT-2 H-Bridge  
**Feedback Sensor:** Potentiometer on pin A9 (0-1023 range)  
**Control Method:** PID with deadband

### Pin Configuration

```cpp
int RPWM_Output = 5;  // Right motor PWM (forward/right turn)
int LPWM_Output = 6;  // Left motor PWM (backward/left turn)
```

**Motor Direction:**
- `RPWM > 0, LPWM = 0`: Steer RIGHT
- `RPWM = 0, LPWM > 0`: Steer LEFT
- `RPWM = 0, LPWM = 0`: NEUTRAL (motor off)

### PID Configuration

```cpp
// PID gains (tune these values)
float steer_kp = 1.0;     // Proportional gain
float steer_ki = 0.0;     // Integral gain
float steer_kd = 0.0;     // Derivative gain

// Limits
const int STEER_DEADBAND = 10;      // Potentiometer deadband
const int STEER_MIN_POT = 0;        // Full left
const int STEER_MAX_POT = 1023;     // Full right
const int STEER_CENTER_POT = 512;   // Center position
```

### PID Algorithm

```cpp
float calculateSteerPID() {
    unsigned long current_time = millis();
    float dt = (current_time - steer_last_time) / 1000.0;
    
    if (dt <= 0) dt = 0.001;  // Prevent division by zero
    
    // Calculate error
    steer_error = steer_setpoint - steer_current;
    
    // Integral term
    steer_error_sum += steer_error * dt;
    
    // Derivative term
    float error_rate = (steer_error - steer_last_error) / dt;
    
    // PID output
    float output = (steer_kp * steer_error) + 
                   (steer_ki * steer_error_sum) + 
                   (steer_kd * error_rate);
    
    steer_last_error = steer_error;
    steer_last_time = current_time;
    
    return output;
}
```

### Steering Control Loop

**Rate:** 10 Hz (every 100ms)

```cpp
void controlSteering() {
    // Read current position
    steer_current = analogRead(STEER_POT_PIN);
    
    // Safety check
    if (!radioStats.signalGood) {
        analogWrite(RPWM_Output, 0);
        analogWrite(LPWM_Output, 0);
        return;
    }
    
    int pwmValue = 0;
    String direction = "NEUTRAL";
    
    switch (radioData.control_mode) {
        case 0: // Pause - hold current position
            analogWrite(RPWM_Output, 0);
            analogWrite(LPWM_Output, 0);
            steer_setpoint = steer_current;
            direction = "PAUSE";
            break;
            
        case 1: // Manual with PID assistance
            steer_setpoint = radioData.steering_val;
            float pid_output = calculateSteerPID();
            
            if (abs(steer_error) <= STEER_DEADBAND) {
                // Within deadband - stop motor
                pwmValue = 0;
                direction = "NEUTRAL";
                analogWrite(RPWM_Output, 0);
                analogWrite(LPWM_Output, 0);
            } else {
                // Outside deadband - apply PID correction
                pwmValue = constrain(abs(pid_output), 0, 255);
                
                if (pid_output > 0) {
                    analogWrite(RPWM_Output, pwmValue);
                    analogWrite(LPWM_Output, 0);
                    direction = "RIGHT";
                } else {
                    analogWrite(RPWM_Output, 0);
                    analogWrite(LPWM_Output, pwmValue);
                    direction = "LEFT";
                }
            }
            break;
            
        case 2: // Auto mode - placeholder for cmd_vel
            steer_setpoint = steer_current;
            analogWrite(RPWM_Output, 0);
            analogWrite(LPWM_Output, 0);
            direction = "AUTO";
            break;
            
        default: // Error - stop motor
            analogWrite(RPWM_Output, 0);
            analogWrite(LPWM_Output, 0);
            direction = "ERROR";
            break;
    }
}
```

**Steering Status Print (every 2 seconds):**
```
1,45690,STEER,mode=1,setpt=512.00,current=510.00,error=2.00,dir=RIGHT,pwm=45
```

### Tuning PID Gains

**Current Settings:** Kp=1.0, Ki=0.0, Kd=0.0 (P-only control)

**Tuning Process:**
1. Start with Kp=1.0, Ki=0.0, Kd=0.0
2. If oscillating: Reduce Kp
3. If slow response: Increase Kp
4. If steady-state error: Add small Ki (0.01-0.1)
5. If overshooting: Add small Kd (0.1-1.0)

**Recommended Starting Points:**
- **Aggressive:** Kp=2.0, Ki=0.1, Kd=0.5
- **Conservative:** Kp=0.5, Ki=0.05, Kd=0.1
- **Current:** Kp=1.0, Ki=0.0, Kd=0.0

---

## Control Modes

### Mode 0: Pause

**Behavior:**
- Transmission: Neutral position
- Steering: Motor off, holds current position
- Radio: Still listening for commands

**Use Case:** Safe idle state

### Mode 1: Manual

**Behavior:**
- Transmission: 10-bucket system controlled by `transmission_val`
- Steering: PID-assisted control following `steering_val`
- Radio: Full control from remote

**Use Case:** Human operator control with PID assistance

**Example Serial Output:**
```
1,45702,TRANS,mode=1,bucket=7,target=2517,current=2520
1,45690,STEER,mode=1,setpt=512.00,current=510.00,error=2.00,dir=RIGHT,pwm=45
```

### Mode 2: Auto (Autonomous)

**Behavior:**
- Transmission: Placeholder for ROS cmd_vel (currently neutral)
- Steering: Placeholder for ROS cmd_vel (currently holds position)
- Radio: Still monitoring for e-stop

**Use Case:** Autonomous navigation (future implementation)

**TODO:** 
- Integrate `cmd_vel` commands from ROS 2
- Map linear velocity to bucket selection
- Map angular velocity to steering setpoint

### Mode 9: Safety Override

**Automatically activated when:**
- Radio signal lost (no data for 2+ seconds)
- E-stop triggered

**Behavior:**
- Transmission: Immediate neutral
- Steering: Motor off
- All outputs safe state

---

## Safety Systems

### E-Stop Relay

**Pin:** 32 (Digital Output)  
**Rate:** 20 Hz (every 50ms)  
**Hardware:** 4-port relay board

```cpp
void estopCheck() {
    if (radioData.estop) {
        digitalWrite(ESTOP_RELAY_PIN, LOW);  // Activate e-stop
    } else {
        digitalWrite(ESTOP_RELAY_PIN, HIGH); // Normal operation
    }
}
```

**Relay States:**
- `HIGH`: Normal operation (relay coil off)
- `LOW`: E-stop activated (relay coil energized, grounds ignition)

### Signal Loss Protection

**Monitor:** `radioStats.signalGood`  
**Timeout:** 2 seconds  
**Action:** Force mode to 9 (safety override)

```cpp
// Checked in every control loop
if (!radioStats.signalGood) {
    radioData.control_mode = 9;  // Safety override
}
```

### Multi-Layer Safety

```
Layer 1: Radio timeout (2 sec) → Mode 9
         ↓
Layer 2: E-stop button → Relay activation
         ↓
Layer 3: Transmission neutral + Steering off
         ↓
Layer 4: Relay grounds ignition circuit
```

---

## Serial Communication Format

### Message Types

The Teensy sends two types of messages over USB serial at 115200 baud:

**Type 1: Status/Logging** (for RPi consumption)
```
1,<timestamp>,<subsystem>,<key1>=<value1>,<key2>=<value2>,...
```

**Type 2: Debug** (for development/troubleshooting)
```
2,<timestamp>,<subsystem>,<message>
```

### Subsystems

| Subsystem | Purpose | Update Rate |
|-----------|---------|-------------|
| **RADIO** | Radio communication status | 5s (data), 10s (stats) |
| **STEER** | Steering control status | 2s |
| **TRANS** | Transmission control status | 5s |
| **TRANS_LOG** | Detailed transmission log | 100ms |
| **JRK** | JRK controller commands | 5s |
| **SYSTEM** | System health/heartbeat | 5s |

### Message Examples

**Radio Data (every 5 seconds):**
```
1,45690,RADIO,steer=512.00,throttle=500.00,trans=469.00,volt=12.4,estop=0,mode=1
```

**Radio Statistics (every 10 seconds):**
```
1,56234,RADIO,ack_rate=4.8,current_rate=4.8,total_acks=48,signal=GOOD,struct_size=28
```

**Steering Status (every 2 seconds):**
```
1,45690,STEER,mode=1,setpt=512.00,current=510.00,error=2.00,dir=RIGHT,pwm=45
```

**Transmission Status (every 5 seconds):**
```
1,45702,TRANS,mode=1,bucket=7,target=2517,current=2520
```

**Transmission Log (every 100ms):**
```
1,45702,TRANS_LOG,output=2520,trans_val=285.50,target=2517,bucket=7,feedback=2518
```

**JRK Target (every 5 seconds):**
```
1,46123,JRK,target=2517.00
```

**System Heartbeat (every 5 seconds):**
```
1,50000,SYSTEM,heartbeat
```

### Debug Messages

```
2,12345,SYSTEM,debug_msg_0
2,12445,SYSTEM,debug_msg_1
2,56789,JRK,timeout_waiting_feedback
2,67890,STEER_POT,raw=512.00
2,67890,STEER_POT,voltage=1.65
```

---

## Timing and Rate Control

### Update Rates Summary

| Function | Rate | Interval | Purpose |
|----------|------|----------|---------|
| **Radio Check** | Variable | ~250Hz max | Check for incoming data |
| **Transmission Control** | 10 Hz | 100ms | Update transmission target |
| **Steering Control** | 10 Hz | 100ms | Update steering PID |
| **E-Stop Check** | 20 Hz | 50ms | Monitor e-stop status |
| **Radio Data Print** | 0.2 Hz | 5s | Log received values |
| **Radio Stats Print** | 0.1 Hz | 10s | Log communication stats |
| **Steering Status Print** | 0.5 Hz | 2s | Log steering status |
| **Trans Status Print** | 0.2 Hz | 5s | Log transmission status |
| **Trans Log Print** | 10 Hz | 100ms | Detailed transmission log |
| **JRK Target Print** | 0.2 Hz | 5s | Log JRK commands |
| **Heartbeat** | 0.2 Hz | 5s | System alive indicator |

### Loop Structure

```cpp
void loop() {
    currentMillis = millis();
    
    handleRadio();           // Check for radio data, send ACKs
    controlTransmission();   // Update transmission (10 Hz)
    controlSteering();       // Update steering PID (10 Hz)
    estopCheck();           // Monitor e-stop (20 Hz)
    
    // Optional: debugSteerPot();  // Debug pot reading (1 Hz)
}
```

**Loop Frequency:** ~250 Hz (4ms per iteration typical)

---

## Startup Sequence

### Boot Delay

```cpp
void setup() {
    delay(45000);  // Wait 45 seconds for RPi to boot
    Serial.begin(115200);
    while (!Serial && millis() < 10000);  // Wait up to 10 more seconds
```

**Total Boot Wait:** Up to 55 seconds to ensure RPi is ready

### Initialization Steps

1. **Serial Communication** (115200 baud)
   ```
   1,500,SYSTEM,teensy_starting
   ```

2. **Debug Message Burst** (10 messages to confirm serial)
   ```
   2,600,SYSTEM,debug_msg_0
   2,700,SYSTEM,debug_msg_1
   ...
   2,1500,SYSTEM,debug_msg_9
   1,1600,SYSTEM,serial_confirmed
   ```

3. **Initialize Hardware**
   - IBT-2 steering pins (5, 6)
   - JRK G2 Serial3 @ 9600 baud
   - SPI bus for NRF24

4. **Initialize NRF24 Radio** (5 attempts with 1s between)
   ```
   1,2000,RADIO,radio_init_attempt_1
   1,2100,RADIO,initialized_success
   ```
   
   If all attempts fail:
   ```
   1,7000,RADIO,hardware_not_responding
   ```

5. **Configure Radio**
   - Set power: `RF24_PA_HIGH`
   - Set data rate: `250KBPS`
   - Set channel: 124
   - Enable ACK payloads
   - Start listening

6. **Initialize E-Stop Relay**
   ```cpp
   pinMode(ESTOP_RELAY_PIN, OUTPUT);
   digitalWrite(ESTOP_RELAY_PIN, HIGH);  // Start safe
   ```

7. **Setup Complete**
   ```
   1,8000,SYSTEM,setup_complete
   1,8100,SYSTEM,bucket_system_ready
   ```

---

## Configuration Parameters

### Transmission Buckets

Modify these values to change speed characteristics:

```cpp
const uint16_t bucketTargets[10] = {
    3696,   // Bucket 0: Full reverse
    3554,   // Bucket 1: 75% reverse
    3412,   // Bucket 2: 50% reverse
    3270,   // Bucket 3: 25% reverse
    3128,   // Bucket 4: Near neutral
    2985,   // Bucket 5: Neutral (MUST MATCH transmissionNeutralPos)
    2751,   // Bucket 6: 25% forward
    2517,   // Bucket 7: 50% forward
    2283,   // Bucket 8: 75% forward
    2048    // Bucket 9: Full forward
};
```

### Bucket Thresholds

Modify these to change when buckets activate:

```cpp
// In controlTransmission(), case 1:
if (transmission_val >= 931)      bucket = 0;
else if (transmission_val >= 838) bucket = 1;
else if (transmission_val >= 746) bucket = 2;
// ... etc
```

### PID Tuning

```cpp
// Steering PID gains
float steer_kp = 1.0;     // Proportional gain
float steer_ki = 0.0;     // Integral gain
float steer_kd = 0.0;     // Derivative gain

// Deadband (prevents oscillation near target)
#define STEER_DEADBAND 10
```

### Radio Configuration

```cpp
radio.setPALevel(RF24_PA_HIGH);      // Power: MIN, LOW, HIGH, MAX
radio.setDataRate(RF24_250KBPS);     // Rate: 250KBPS, 1MBPS, 2MBPS
radio.setChannel(124);                // Channel: 0-125
```

### Timing Intervals

```cpp
// Radio
const unsigned long signalTimeout = 2000;       // 2 seconds
const unsigned long rateReportInterval = 10000; // 10 seconds
const unsigned long dataPrintInterval = 5000;   // 5 seconds

// Control loops
const unsigned long controlTransmissionInterval = 100; // 10 Hz
const unsigned long controlSteeringInterval = 100;     // 10 Hz
const unsigned long estopCheckInterval = 50;           // 20 Hz

// Print intervals
const unsigned long targetPrintInterval = 5000;     // 5 seconds
const unsigned long steeringPrintInterval = 2000;   // 2 seconds
```

---

## Troubleshooting

### Radio Not Initializing

**Symptoms:**
```
1,2000,RADIO,radio_init_attempt_1
1,3000,RADIO,radio_init_attempt_2
...
1,7000,RADIO,hardware_not_responding
```

**Possible Causes:**
1. Wiring issue (CE, CSN, SPI pins)
2. NRF24 module not powered
3. Bad NRF24 module
4. SPI pin conflict

**Debug Steps:**
```cpp
// Add to setup() after SPI.begin():
Serial.print("SPI SCK: "); Serial.println(13);
Serial.print("SPI MOSI: "); Serial.println(11);
Serial.print("SPI MISO: "); Serial.println(12);
```

### No Radio Data Received

**Symptoms:**
- No RADIO messages appearing
- `signal=LOST` in statistics
- Mode automatically switching to 9

**Check:**
1. Radio control unit is powered and transmitting
2. Both units on same channel (124)
3. Both units using same addresses
4. Distance < 100 meters (clear line of sight)

**Debug:**
```cpp
// Uncomment in handleRadio():
if (radio.available()) {
    Serial.println("DEBUG: Radio packet received!");
}
```

### Steering Not Responding

**Symptoms:**
```
1,45690,STEER,mode=1,setpt=512.00,current=510.00,error=2.00,dir=NEUTRAL,pwm=0
```

**Possible Causes:**
1. Error within deadband (normal behavior if error < 10)
2. IBT-2 not powered
3. Steering motor issue
4. PID gains too low

**Debug:**
```cpp
// Uncomment in loop():
debugSteerPot();  // Prints pot value every 1 second
```

Expected output:
```
2,67890,STEER_POT,raw=512.00
2,67890,STEER_POT,voltage=1.65
```

**Check Pot Wiring:**
- Pin A9 (physical pin 23) connected
- Pot voltage between 0-3.3V
- Pot value changes when wheels turned

### Transmission Not Moving

**Symptoms:**
```
1,45702,TRANS,mode=1,bucket=7,target=2517,current=2985
```
(Target changes but current stays at neutral)

**Possible Causes:**
1. JRK G2 not powered
2. Serial3 wiring issue
3. Linear actuator disconnected
4. JRK G2 not configured

**Debug:**
```cpp
// Check JRK feedback
uint16_t feedback = readFeedback();
if (feedback == 0xFFFF) {
    Serial.println("ERROR: JRK not responding");
}
```

**Verify JRK Configuration:**
- Input mode: Serial
- Baud rate: 9600
- Feedback: Analog
- Scaling: Appropriate for actuator

### E-Stop Not Working

**Symptoms:**
- Relay not clicking
- Engine not shutting down when estop=1

**Check:**
1. Relay pin 32 wiring
2. Relay power supply
3. Ignition circuit wiring to relay NO/NC

**Debug:**
```cpp
// Add to estopCheck():
Serial.print("E-Stop pin state: ");
Serial.println(digitalRead(ESTOP_RELAY_PIN) ? "HIGH" : "LOW");
```

### High Serial Data Volume

**Symptom:** Serial monitor overwhelmed with messages

**Solution:** Comment out high-frequency logging:

```cpp
// In controlTransmission(), comment out:
// printStatusMultiKV("TRANS_LOG");  // 10 Hz - very verbose
```

---

## Performance Characteristics

### Memory Usage

```
Sketch uses 91,824 bytes (35%) of program storage space. Maximum is 262,144 bytes.
Global variables use 8,952 bytes (13%) of dynamic memory, leaving 56,584 bytes for local variables.
```

### CPU Usage

- Main loop: ~250 Hz (4ms per iteration)
- Radio checks: ~250 Hz when active
- Control loops: 10-20 Hz (light load)
- **CPU Load:** <10% typical

### Communication Rates

- **Radio RX:** ~5 Hz typical (limited by control unit TX rate)
- **Radio ACK:** Matches RX rate
- **Serial TX:** ~30 messages/second typical
  - TRANS_LOG: 10 Hz
  - Status prints: Variable (1-10 Hz total)

---

## Future Enhancements

### Autonomous Navigation Integration

**TODO:** Add cmd_vel input from ROS 2

```cpp
// Proposed structure for ROS commands
struct CmdVelStruct {
    float linear_x;   // -1.0 to +1.0 m/s
    float angular_z;  // -1.0 to +1.0 rad/s
};

// In controlTransmission(), case 2:
// Map linear_x to bucket selection
// Example: linear_x=0.5 → bucket 7

// In controlSteering(), case 2:
// Map angular_z to steer_setpoint
// Example: angular_z=0.3 → setpoint=650
```

### Improved PID Tuning

**TODO:** Add auto-tuning capability or external parameter updates

```cpp
// Receive PID gains via serial
// Format: "PID,kp,ki,kd"
if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.startsWith("PID,")) {
        // Parse and update gains
    }
}
```

### Encoder Integration

**TODO:** Add wheel encoder feedback for odometry

```cpp
// Left and right wheel encoders
// Integration with Pure Pursuit navigation
```

---

## Related Documentation

- **Teensy Serial Bridge:** See `teensy_serial_bridge_documentation.md`
- **Radio Control Unit:** See radio control firmware documentation
- **System Architecture:** See main project `README.md`
- **Hardware Pinouts:** See Teensy 3.5 pinout diagram

---

## Changelog

### Version 1.0 (January 2025)
- 10-bucket transmission system
- PID-based steering control
- Consolidated radio handling
- Standardized serial message format
- Multi-layer safety systems
- Comprehensive status logging

---

## License

Part of the Tractor2025 autonomous platform project.

## Contact

For questions or issues: aej2126 _at_ protonmail !dot! com