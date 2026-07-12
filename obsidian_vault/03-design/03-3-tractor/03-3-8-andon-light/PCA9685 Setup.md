Setup

![[Exported image 20260511102556-0.png]]     
![[Exported image 20260511102558-1.png]]  
![[Exported image 20260511102559-2.png]]  

|   |
|---|
|Enable I²C on Ubuntu (RPi 5)<br><br>  <br><br>Edit the config file<br><br>  <br><br>sudo nano /boot/firmware/config.txt<br><br>  <br>  <br><br>Add this line at the bottom:<br><br>  <br><br>dtparam=i2c_arm=on  <br>Save and Exit<br><br>  <br><br>Enable I²C kernel modules: $ echo "i2c-dev" \| sudo tee -a /etc/modules  <br>Reboot: $ sudo reboot<br><br>  <br><br>Install tools  <br>$ sudo apt update  <br>$ sudo apt install -y i2c-tools python3-smbus<br><br>  <br><br>To test I²C bus, after reboot, run: $ ls /dev/i2c*<br><br>  <br><br>You should see: /dev/i2c-1<br><br>  <br><br>Then scan for devices: $ sudo i2cdetect -y 1<br><br>  <br>  <br><br>You should see 40 (the PCA9685 default address) show up.|
 
|   |
|---|
|sudo apt update  <br>sudo apt install -y python3-pip python3-smbus i2c-tools  <br>sudo apt install -y python3-lgpio|
 
Test script

|   |
|---|
|import board  <br>import busio  <br>from adafruit_pca9685 import PCA9685<br><br>  <br><br>print("🔍 Checking PCA9685 connection...")<br><br>  <br><br># Setup I2C  <br>i2c = busio.I2C(board.SCL, board.SDA)<br><br>  <br><br># Initialize PCA9685  <br>pca = PCA9685(i2c, address=0x40) # Default address<br><br>  <br><br># Set frequency (just to confirm config works)  <br>pca.frequency = 50<br><br>  <br><br>print("✅ PCA9685 detected at address 0x40")  <br>print(" PWM frequency set to:", pca.frequency, "Hz")<br><br>  <br><br># Clean up  <br>pca.deinit()  <br>print("ℹ️ PCA9685 deinitialized, test complete.")|
    
12/24/25

![[Exported image 20260511102600-3.png]]