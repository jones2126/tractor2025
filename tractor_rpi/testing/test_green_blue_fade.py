#!/usr/bin/env python3
# Standalone test: Fade GREEN(9) + BLUE(10) together, continuous cycle.
# Matches your example: Up 0-100% (1s), down 100-0% (1s), repeat ~2s full cycle.
# Run: python3 test_green_blue_fade.py (Ctrl+C to stop).

import time
import board
import busio
from adafruit_pca9685 import PCA9685

# Channels (your wiring)
CH_GREEN = 9
CH_BLUE = 10

# Inversion (from your code)
INVERT_GREEN = False
INVERT_BLUE = False

# Helpers (from your example)
def pct_to_16bit(pct: int, invert: bool) -> int:
    pct = max(0, min(100, int(pct)))
    if invert:
        pct = 100 - pct
    return int(round((pct * 65535) / 100))

def set_channel_percent(pca: PCA9685, ch: int, pct: int, invert: bool):
    pca.channels[ch].duty_cycle = pct_to_16bit(pct, invert)

def all_off(pca: PCA9685):
    set_channel_percent(pca, CH_GREEN, 0, INVERT_GREEN)
    set_channel_percent(pca, CH_BLUE, 0, INVERT_BLUE)

def fade_dual(pca: PCA9685):
    """Fade GREEN + BLUE together: Up then down, repeat."""
    print("Starting GREEN + BLUE dual-fade (up 1s, down 1s, repeat)...")
    step_delay = 0.02  # 20ms/step for ~50 steps up/down (smooth 1s ramp)
    num_steps = 50
    for cycle in range(10):  # 10 cycles for test
        print(f"Cycle {cycle+1}/10: Up...")
        for i in range(num_steps + 1):
            pct = int((i / num_steps) * 100)
            set_channel_percent(pca, CH_GREEN, pct, INVERT_GREEN)
            set_channel_percent(pca, CH_BLUE, pct, INVERT_BLUE)
            time.sleep(step_delay)
        print("Down...")
        for i in range(num_steps, -1, -1):
            pct = int((i / num_steps) * 100)
            set_channel_percent(pca, CH_GREEN, pct, INVERT_GREEN)
            set_channel_percent(pca, CH_BLUE, pct, INVERT_BLUE)
            time.sleep(step_delay)
    print("Test complete—check if both faded smoothly!")

def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 500  # Your freq
    try:
        all_off(pca)
        fade_dual(pca)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        all_off(pca)
        pca.deinit()

if __name__ == "__main__":
    main()