#!/usr/bin/env python3
# PCA9685 fade: RED(8) -> GREEN(9) -> BLUE(10) -> YELLOW(12+11) -> repeat
# Mirrors your Arduino behavior on a Raspberry Pi 5.

import time
import sys
import math

import board
import busio
from adafruit_pca9685 import PCA9685

# ---------- Channel map ----------
CH_RED   = 8    # standalone Red
CH_GREEN = 9    # standalone Green
CH_BLUE  = 10   # standalone Blue
CH_YR    = 12   # Yellow's Red component (100%)
CH_YG    = 11   # Yellow's Green component (90%)

# ---------- Inversion flags (set True if DIM is low-active) ----------
INVERT_RED   = False
INVERT_GREEN = False
INVERT_BLUE  = False
INVERT_YR    = False
INVERT_YG    = False

# ---------- Yellow balance (you liked 100% R, 90% G) ----------
YELLOW_GAIN_R = 1.00
YELLOW_GAIN_G = 0.90

# ---------- Timing ----------
STEP_DELAY_MS = 40    # ms per brightness step
PAUSE_BETWEEN = 0.300 # seconds between colors
FREQ_HZ       = 500   # PCA9685 PWM frequency (300–800 is a good range)

# ---------- Helpers ----------
# PCA9685 uses 16-bit duty cycle: 0..65535 (0%..100%)
def pct_to_16bit(pct: int, invert: bool) -> int:
    pct = max(0, min(100, int(pct)))
    if invert:
        pct = 100 - pct
    return int(round((pct * 65535) / 100))

def set_channel_percent(pca: PCA9685, ch: int, pct: int, invert: bool):
    pca.channels[ch].duty_cycle = pct_to_16bit(pct, invert)

def all_off(pca: PCA9685):
    set_channel_percent(pca, CH_RED,   0, INVERT_RED)
    set_channel_percent(pca, CH_GREEN, 0, INVERT_GREEN)
    set_channel_percent(pca, CH_BLUE,  0, INVERT_BLUE)
    set_channel_percent(pca, CH_YR,    0, INVERT_YR)
    set_channel_percent(pca, CH_YG,    0, INVERT_YG)

# Only the yellow pair on; others off.
def set_yellow_percent(pca: PCA9685, pct: int):
    pct = max(0, min(100, int(pct)))
    r_pct = int(round(pct * YELLOW_GAIN_R))
    g_pct = int(round(pct * YELLOW_GAIN_G))
    r_pct = 100 if r_pct > 100 else r_pct
    g_pct = 100 if g_pct > 100 else g_pct

    # Drive yellow components
    set_channel_percent(pca, CH_YR, r_pct, INVERT_YR)
    set_channel_percent(pca, CH_YG, g_pct, INVERT_YG)

    # Ensure standalone RGB off
    set_channel_percent(pca, CH_RED,   0, INVERT_RED)
    set_channel_percent(pca, CH_GREEN, 0, INVERT_GREEN)
    set_channel_percent(pca, CH_BLUE,  0, INVERT_BLUE)

# Only one standalone color on; others off (including yellow pair).
def set_solo_color(pca: PCA9685, ch_on: int, pct: int, inv_on: bool):
    # Turn requested channel on
    set_channel_percent(pca, ch_on, pct, inv_on)
    # Turn other singles off
    if ch_on != CH_RED:
        set_channel_percent(pca, CH_RED, 0, INVERT_RED)
    if ch_on != CH_GREEN:
        set_channel_percent(pca, CH_GREEN, 0, INVERT_GREEN)
    if ch_on != CH_BLUE:
        set_channel_percent(pca, CH_BLUE, 0, INVERT_BLUE)
    # Yellow pair always off here
    set_channel_percent(pca, CH_YR, 0, INVERT_YR)
    set_channel_percent(pca, CH_YG, 0, INVERT_YG)

def fade_solo(pca: PCA9685, ch: int, inv: bool, label: str):
    print(f"Fading {label}")
    for p in range(0, 101):
        set_solo_color(pca, ch, p, inv)
        time.sleep(STEP_DELAY_MS / 1000.0)
    for p in range(100, -1, -1):
        set_solo_color(pca, ch, p, inv)
        time.sleep(STEP_DELAY_MS / 1000.0)
    time.sleep(PAUSE_BETWEEN)

def fade_yellow(pca: PCA9685, label: str):
    print(f"Fading {label} (R={YELLOW_GAIN_R:.2f}, G={YELLOW_GAIN_G:.2f})")
    for p in range(0, 101):
        set_yellow_percent(pca, p)
        time.sleep(STEP_DELAY_MS / 1000.0)
    for p in range(100, -1, -1):
        set_yellow_percent(pca, p)
        time.sleep(STEP_DELAY_MS / 1000.0)
    time.sleep(PAUSE_BETWEEN)

def main():
    # I2C + PCA9685 init
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = FREQ_HZ

    print("PCA9685 Fade: RED(8) -> GREEN(9) -> BLUE(10) -> YELLOW(12+11) -> repeat")
    print(f"PWM frequency: {FREQ_HZ} Hz")
    try:
        all_off(pca)  # ensure clean start
        while True:
            fade_solo(pca, CH_RED,   INVERT_RED,   "RED (CH8)")
            fade_solo(pca, CH_GREEN, INVERT_GREEN, "GREEN (CH9)")
            fade_solo(pca, CH_BLUE,  INVERT_BLUE,  "BLUE (CH10)")
            fade_yellow(pca, "YELLOW (CH12 + CH11)")
    except KeyboardInterrupt:
        print("\nStopping... turning all channels OFF.")
    finally:
        all_off(pca)
        pca.deinit()

if __name__ == "__main__":
    main()
