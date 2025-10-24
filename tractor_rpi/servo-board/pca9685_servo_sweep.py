# pca9685_sweep_0_1_2.py
import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

SWEEP_DURATION_SEC = 20.0   # per channel
STEP_DEG = 5                # angular step (smaller = smoother)

def angle_sequence(step=5):
    up = list(range(0, 181, step))
    down = list(range(180, -1, -step))
    return up + down

# Setup I2C + PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # standard servo frequency

# Create servo objects for channels 0,1,2 (wider pulse window helps cheap servos)
servos = {
    0: servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2500),
    1: servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2500),
    2: servo.Servo(pca.channels[2], min_pulse=500, max_pulse=2500),
}

seq = angle_sequence(STEP_DEG)
delay = SWEEP_DURATION_SEC / len(seq)

print(f"Sweeping channels 0, 1, 2; each sweep = {SWEEP_DURATION_SEC:.1f}s, step = {STEP_DEG}°")

try:
    while True:
        for ch in (0, 1, 2):
            s = servos[ch]
            print(f"\n--- Channel {ch} sweep start ---")
            for a in seq:
                s.angle = a
                time.sleep(delay)
            print(f"--- Channel {ch} sweep complete ---")
except KeyboardInterrupt:
    print("\nStopping; releasing servos.")
finally:
    # Optionally release servos and deinit PWM
    for s in servos.values():
        try:
            s.angle = None
        except Exception:
            pass
    try:
        pca.deinit()
    except Exception:
        pass
