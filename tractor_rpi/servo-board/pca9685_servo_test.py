# servo_verify_loop.py
import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16, address=0x40)
s = kit.servo[2]
s.set_pulse_width_range(500, 2500)  # widen range for cheap servos

print("Starting servo verify on channel 0 (angles: 90 → 0 → 180 → 90 → 60 → 120 → 90)")
angles = [90, 0, 180, 90, 60, 120, 90]

i = 0
try:
    while True:
        angle = angles[i % len(angles)]
        print(f"→ Setting angle to {angle}°")
        s.angle = angle
        time.sleep(1.0)  # hold each position
        i += 1
except KeyboardInterrupt:
    print("\nStopping; releasing servo.")
    s.angle = None
    # Optional: deinit the PCA9685
    try:
        kit._pca.deinit()
    except Exception:
        pass
