# BIG7 USB Hub (Rev 2) — RPi 5 Connection Guide

Reference manual: [[BIG7_Rev2_UserManual.pdf]]

The manual was written for RPi 3 and earlier. The RPi 5 setup uses Self-Power Mode with two separate power supplies — one for the hub, one for the Pi. No pogo pins needed.

## What You Need

- Buck converter with 5V USB output (dedicated 12V → 5V)
- Micro-USB cable (hub power)
- USB-A to Mini-USB cable (data link between Pi and hub)

## Step 1 — Set the Power Link Jumper to "Non Power Link"

Find the small jumper labeled **Power Link** (item 9, right side of board near the upstream mini-USB port). Move it to the **opposite** side of "Power Link." This prevents the hub from trying to back-power the RPi, keeping the two supplies isolated.

## Step 2 — Power the BIG7 from the Buck Converter

Plug the buck converter's 5V USB output into the **Micro-USB port on the BIG7** (item 10, lower-left of board). The red LED (item 11) should light up.

This is Self-Power Mode — up to **3A total** across all 7 ports.

## Step 3 — Connect Data from RPi 5 to the BIG7

Use a **USB-A to Mini-USB cable**:
- USB-A end → any USB port on the RPi 5
- Mini-USB end → **Upstream USB port** on the BIG7 (item 8, right side of board)

## Step 4 — Power the RPi 5 Normally

Power the RPi 5 via its own USB-C supply as usual.

## Wiring Summary

```
Buck Converter (12V→5V)
    └── Micro-USB ──→ BIG7 power-in (item 10)       [powers hub]

RPi 5 USB-C supply
    └── USB-C ──→ RPi 5                              [powers Pi]

RPi 5 USB-A port
    └── USB-A to Mini-USB ──→ BIG7 upstream (item 8) [data]

BIG7 downstream ports 1–7
    └── Teensy, GPS ×2, OAK-D, etc.
```

## Notes

- The BIG7 is USB 2.0 only (480 Mbps max). Plug the data cable into any RPi 5 USB port — it will negotiate at USB 2.0 speed, which is fine for all connected devices.
- Self-Power mode max output: 3A across all 7 ports combined.
- The power link jumper **must** be on "Non Power Link" when using two separate supplies — leaving it on "Power Link" with two supplies can cause damage.
