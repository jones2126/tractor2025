![[Exported image 20260511103734-0.png]]   
**1. Hardware Setup**

- Connected the **LACT4P actuator** motor leads to **A/B** terminals on the JRK G2.
- Connected a regulated **12VDC power supply** to **VIN/GND** on the JRK.
- Connected the **JRK to Windows 10 PC via USB**.
- Installed and launched the **Pololu JRK G2 Configuration Utility**.

**2. Tested Actuator Using USB**

- Verified JRK detected in config software.
- Used the **Status tab target slider** to move actuator (after resolving VIN power issue).
- Confirmed actuator movement and feedback behavior.

**3. Configured Feedback**

- Set **Feedback mode** to **Analog voltage** under the **Feedback** tab.
- Used **"Feedback setup wizard…"** to record actuator’s analog range:
    - Moved actuator to full retraction and extension.
    - Wizard set calibrated **min** and **max** values (e.g., 344 to 3668).
- Verified feedback updates with actuator motion.

**4. Configured PID Control**

- Opened **PID tab** and confirmed reasonable defaults:
    - P = 5.0, I ≈ 0.1, D ≈ 0.1
- Applied settings to enable closed-loop position control.
- Verified actuator moves and holds position using slider in Status tab.

**5. Configured Serial Communication**

- Set **Input mode** to **“Serial / I²C / USB”** in **Input** tab.
- Set **Serial interface**:
    - Mode: UART, fixed baud rate
    - Baud rate: 9600 (or 115200, match Teensy)
    - Leave unchecked:
        - Enable CRC
        - Enable 14-bit device number
        - Disable compact protocol
    - Leave **device number = 11** (default)
- ![[Exported image 20260511103736-1.png]]
- Saved settings to JRK.

**6. Wired Teensy 3.5 to JRK G2 (Serial3)**

|   |   |   |
|---|---|---|
|**JRK G2 Pin**|**Teensy 3.5 Pin**|**Description**|
|RX|TX3 (pin 8)|Teensy → JRK|
|TX|RX3 (pin 7)|JRK → Teensy (optional)|
|GND|GND|Shared ground|
 
|   |
|---|
|```<br>#include \<Arduino.h\><br>unsigned long jrkTarget = 0;<br>unsigned long jrkPause = 10000;<br>void setTarget(uint16_t target) {<br>  Serial3.write(0xAA);               // Start byte<br>  Serial3.write(11);                 // Device number<br>  Serial3.write(0x40);               // Set Target command<br>  Serial3.write(target & 0x1F);      // 5 LSBs<br>  Serial3.write((target \>\> 5) & 0x7F); // 7 MSBs<br>}<br>void setTargetCompact(uint16_t target) {<br>  if (target \> 4095) target = 4095;  // Clamp range<br>  Serial3.write(0xC0 + (target & 0x1F));       // Command byte with 5 LSBs<br>  Serial3.write((target \>\> 5) & 0x7F);         // 7 MSBs<br>}<br>void setup() {<br>  delay(45000);  // delay to let RPi boot up so Serial will work<br>  Serial.begin(115200);      // Serial monitor for debugging<br>  Serial3.begin(9600); // Match JRK baud rate<br>}<br>void loop() {<br>  jrkTarget = 2048; // Midpoint<br>  Serial.println("Moving to " + String(jrkTarget));<br>  // setTarget(jrkTarget); delay(jrkPause); // Midpoint<br>  setTargetCompact(jrkTarget); delay(jrkPause); // Midpoint<br>  jrkTarget = 4095; // Fully Extend<br>  Serial.println("Moving to " + String(jrkTarget));<br>  // setTarget(jrkTarget); delay(jrkPause);  <br>  setTargetCompact(jrkTarget); delay(jrkPause); <br>  jrkTarget = 0; // Retract<br>  Serial.println("Moving to " + String(jrkTarget));<br>  // setTarget(jrkTarget); delay(jrkPause);<br>  setTargetCompact(jrkTarget); delay(jrkPause); <br>}<br>I could not get the setTarget function to work using the full protocol, only the compact protocol<br>```|
 
Code to use the compact protocol and another command to read the position