import board
import busio
from adafruit_pca9685 import PCA9685

print("🔍 Checking PCA9685 connection...")

# Setup I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize PCA9685
pca = PCA9685(i2c, address=0x40)  # Default address

# Set frequency (just to confirm config works)
pca.frequency = 50

print("✅ PCA9685 detected at address 0x40")
print("   PWM frequency set to:", pca.frequency, "Hz")

# Clean up
pca.deinit()
print("ℹ️ PCA9685 deinitialized, test complete.")
