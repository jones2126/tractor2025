# LED Status Tower - Quick Reference Guide

## At a Glance

```
┌─────────────────────────────────────────────────┐
│  🔴 RED LED    - Safety & E-Stop Status         │
│  🟡 YELLOW LED - GPS/RTK Positioning            │
│  🟢 GREEN LED  - System Health                  │
│  🔵 BLUE LED   - Control Mode & Communication   │
└─────────────────────────────────────────────────┘
```

## Boot Sequence (First 10 seconds)
```
🟡 Yellow: Slow fading
🟢 Green:  Slow blinking (0.5Hz)
🔵 Blue:   OFF
🔴 Red:    Slow blinking (e-stop armed)
```

## Ready to Operate (Manual Mode)
```
🟡 Yellow: 3Hz blinking (GPS) OR solid (RTCM) OR 5Hz blinking (RTK Fixed)
🟢 Green:  Solid ON
🔵 Blue:   1Hz blinking (NRF24) OR 2Hz blinking (WiFi)
🔴 Red:    Slow blinking (1Hz) = e-stop armed
```

## Autonomous Mode - READY
```
🟡 Yellow: Fast blinking (5Hz) - RTK Fixed ⭐
🟢 Green:  Solid ON
🔵 Blue:   Solid ON
🔴 Red:    Slow blinking (1Hz)
```
**✅ Safe to start autonomous mission**

## Autonomous Mode - PAUSED (RTK Lost)
```
🟡 Yellow: Alternating bright/dim ⚠️
🟢 Green:  Solid ON
🔵 Blue:   Fast blinking (5Hz) ⚠️
🔴 Red:    Slow blinking (1Hz)
```
**⚠️ Take manual control immediately!**

## EMERGENCY STOP ACTIVE
```
🟡 Yellow: Any state
🟢 Green:  Any state  
🔵 Blue:   Any state
🔴 Red:    SOLID ON 🛑
```
**🛑 System halted - investigate before reset**

## Troubleshooting Quick Checks

### Problem: Yellow LED still fading after 30 seconds
```
Issue: No GPS fix
Check: 
  - GPS antenna connection
  - Clear view of sky
  - GPS power supply
```

### Problem: Yellow LED blinking, but won't go solid
```
Issue: No RTCM corrections
Check:
  - Base station connection
  - Network connectivity
  - RTCM data stream
```

### Problem: Blue LED is OFF
```
Issue: No communication
Check:
  - NRF24 radio battery
  - WiFi connection
  - Radio control power
```

### Problem: Green LED blinking fast (5Hz)
```
Issue: Critical system warning
Check:
  - Battery voltage
  - System logs
  - Temperature
```

### Problem: Red LED solid ON
```
Issue: E-STOP triggered
Actions:
  1. Identify cause of e-stop
  2. Resolve safety issue
  3. Reset e-stop switch
  4. Verify system status before resuming
```

## Color-Coded Status Summary

### 🟡 Yellow LED Patterns
| Pattern | Meaning |
|---------|---------|
| Fade (slow) | No GPS yet |
| Blink 3Hz | GPS active, no RTCM |
| Solid | RTCM corrections active |
| Blink 5Hz | ⭐ RTK Fixed (best) |
| Alternating | ⚠️ RTK lost |

### 🔵 Blue LED Patterns
| Pattern | Meaning |
|---------|---------|
| OFF | ⚠️ No communication |
| Blink 1Hz | Manual + NRF24 |
| Blink 2Hz | Manual + WiFi |
| Solid | ⭐ Auto + RTK ready |
| Blink 5Hz | ⚠️ Auto paused |

### 🟢 Green LED Patterns
| Pattern | Meaning |
|---------|---------|
| OFF | System off |
| Blink 0.5Hz | Booting |
| Solid | ⭐ All good |
| Blink 2Hz | ⚠️ Warning |
| Blink 5Hz | ⚠️ Critical |

### 🔴 Red LED Patterns
| Pattern | Meaning |
|---------|---------|
| OFF | No errors |
| Blink 1Hz | E-stop armed |
| Blink 5Hz | ⚠️ Safety issue |
| Solid | 🛑 E-STOP ACTIVE |

## Pre-Operation Checklist

Before starting any operation, verify:

- [ ] 🟢 Green LED: Solid (system healthy)
- [ ] 🔴 Red LED: Slow blink (e-stop armed, not triggered)
- [ ] 🟡 Yellow LED: Minimum 3Hz blink (GPS active)
- [ ] 🔵 Blue LED: Blinking (communication active)

**For Autonomous Operations, additionally verify:**
- [ ] 🟡 Yellow LED: Fast 5Hz blink (RTK fixed)
- [ ] 🔵 Blue LED: Solid (auto mode ready)

## Emergency Procedures

### If RTK is Lost During Mission (Yellow alternating, Blue fast blink)
1. Robot should automatically pause
2. Switch to manual control immediately
3. Drive to safe location
4. Wait for RTK to recover (Yellow back to 5Hz blink)
5. Resume mission or return to base

### If Communication is Lost (Blue LED goes OFF)
1. Robot should automatically stop
2. Approach robot location
3. Check radio/WiFi connection
4. Re-establish communication before resuming
5. If can't reconnect, use manual e-stop

### If E-Stop is Triggered (Red LED solid)
1. System is halted
2. Identify what triggered e-stop:
   - Manual button pressed?
   - Communication timeout?
   - Safety violation detected?
3. Resolve the issue
4. Reset e-stop
5. Verify all LEDs return to normal before resuming

## Contact Information

**For Technical Support:**
- GitHub: https://github.com/jones2126/tractor2025
- Slack: Lawn Tractor Automation
- Email: aej2126@protonmail.com

**Emergency Shutdown:**
1. Press manual e-stop button on robot
2. Turn off main power if accessible
3. Remove battery if necessary

---

**Remember:** When in doubt, press the e-stop button!

Print this page and keep it near your robot control station.
