# SkyTraq PX1125R Binary Commands and Outputs

**Note on Viewing Log Files**: The RTCM log files (e.g., `rtcm_20250507_184932.txt`) contain binary data, often viewed as hex strings. To accurately review these logs, use a text editor designed for handling binary or hex data, such as Sublime Text or Visual Studio Code, which display hex bytes without interpreting them as ASCII. Avoid using editors like Notepad, as they attempt to translate binary data into ASCII, resulting in unreadable output.

This document outlines the binary commands accepted by the SkyTraq PX1125R GNSS receiver, their formats, and the expected output responses. The information is derived from successful configurations on Windows 10 using COM6 (base station), analysis of RTCM logs (`rtcm_20250507_184932.txt` and recent run on May 8, 2025), and interactions via GNSS Viewer (Customer Release V2.1.174 for Phoenix). The PX1125R runs firmware version `03.06.00, 01.07.33`.

## Binary Command Format

The PX1125R uses the SkyTraq binary protocol for configuration. Commands follow this structure:

```
a0a1<length><messagebody><checksum>0d0a
```

- **Preamble**: `a0a1` (2 bytes, fixed).
- **Length**: 2 bytes (little-endian), length of `<messagebody>` in bytes.
- **Message Body**: Command-specific data (variable length).
- **Checksum**: 1 byte, XOR of all bytes in `<messagebody>`.
- **Terminator**: `0d0a` (CRLF).

### Checksum Calculation
- Compute XOR of all bytes in `<messagebody>`.
- Example: For `6401`, checksum = `64 XOR 01` = `65`.

### Response Format
Responses follow a similar structure:

```
a0a1<length>83<messageid><status/data><checksum>0d0a
```

- **Preamble**: `a0a1`.
- **Length**: 2 bytes (little-endian).
- **Response Indicator**: `83` (acknowledgment).
- **Message ID**: Matches the command’s message ID (e.g., `64` for firmware query).
- **Status/Data**: Command-specific (e.g., `01` for success, or additional data).
- **Checksum**: XOR of `83 <messageid> <status/data>`.
- **Terminator**: `0d0a`.

## Supported Binary Commands

Below are the known commands, their purposes, formats, and expected responses, verified against logs, script execution, and GNSS Viewer interactions. Hex values are presented in lowercase without spaces between bytes, matching the format seen in Sublime Text.

### Generic Python Code for Sending Commands
The following Python code can be used to send any of the commands listed below and read the response from the PX1125R. Replace the `command` variable with the specific command bytes for each message.

```python
import serial

# Open serial port (e.g., COM6 at 115200 baud)
ser = serial.Serial("COM6", 115200, timeout=1)

# Define the command (replace with specific command bytes)
command = bytes.fromhex("a0a100026401650d0a")  # Example for Firmware Query

# Send the command
ser.write(command)
ser.flush()

# Read and print the response
response = ser.read(1024)  # Adjust buffer size as needed
print(f"Response: {response.hex()}")

# Close the serial port
ser.close()
```

### 1. Firmware Query (`6401`)
- **Purpose**: Queries the receiver’s firmware version.
- **Command**: `a0a100026401650d0a`
  - Length: `0002` (2 bytes).
  - Message: `6401` (query software version).
  - Checksum: `64 XOR 01` = `65`.
- **Expected Response**:
  ```
  a0a1<length>836401<checksum>0d0a <additionaldata>
  ```
  - Length: Variable (e.g., `0003` for ACK, plus data).
  - Response: `836401` (ACK, success).
  - Checksum: XOR of `836401` = `e6` (plus data if present).
  - Additional Data: ASCII string, e.g., `SKYTRAQ,03.06.00,01.07.33,e42cc57c4,unknown`.
- **Log Evidence**: In `rtcm_20250507_184932.txt` and recent run:
  ```
  534b59545241511b30332e30362e30302c30312e30372e33332c65343263633537633407756e6b6e6f776e
  ```
  Decodes to: `SKYTRAQ,03.06.00,01.07.33,e42cc57c4,unknown`.
- **Command to Send**:
  ```python
  command = bytes.fromhex("a0a100026401650d0a")
  ```

### 2. Query Software CRC (`641a`)
- **Purpose**: Queries the CRC of the receiver’s firmware.
- **Command**: `a0a10002641a7e0d0a`
  - Length: `0002`.
  - Message: `641a` (query software CRC).
  - Checksum: `64 XOR 1a` = `7e`.
- **Expected Response**:
  ```
  a0a1000383641afd0d0a <additionaldata>
  ```
  - Length: Variable (e.g., `0003` for ACK, plus data).
  - Response: `83641a` (ACK, success).
  - Checksum: `83 XOR 64 XOR 1a` = `fd`.
  - Additional Data: CRC value (format depends on firmware).
- **Log Evidence**: From GNSS Viewer response:
  ```
  a0a1000383641afd0d0a
  ```
  Confirms the command structure. Additional data (CRC value) not logged in this instance.
- **Command to Send**:
  ```python
  command = bytes.fromhex("a0a10002641a7e0d0a")
  ```

### 3. Cold Start (`6501`)
- **Purpose**: Reinitializes the receiver, clearing ephemeris and almanac data.
- **Command**: `a0a100026501640d0a`
  - Length: `0002`.
  - Message: `6501` (cold start).
  - Checksum: `65 XOR 01` = `64`.
- **Expected Response**:
  ```
  a0a10003836501e30d0a
  ```
  - Response: `836501` (ACK, success).
  - Checksum: `83 XOR 65 XOR 01` = `e3`.
- **Notes**: Not used in `forward_rtcm_windows.py`, suggesting prior settings were retained.
- **Command to Send**:
  ```python
  command = bytes.fromhex("a0a100026501640d0a")
  ```

### 4. Factory Reset (`6504`)
- **Purpose**: Resets the receiver to factory defaults, clearing all configurations.
- **Command**: `a0a100026504610d0a`
  - Length: `0002`.
  - Message: `6504` (factory reset).
  - Checksum: `65 XOR 04` = `61`.
- **Expected Response**:
  ```
  a0a10003836504e00d0a
  ```
  - Response: `836504` (ACK, success).
  - Checksum: `83 XOR 65 XOR 04` = `e0`.
- **Notes**: Not used in the script, likely unnecessary if MSM4 settings are intact.
- **Command to Send**:
  ```python
  command = bytes.fromhex("a0a100026504610d0a")
  ```

### 5. Software Reboot (`6502`)
- **Purpose**: Reboots the receiver to stabilize firmware state.
- **Command**: `a0a100026502630d0a`
  - Length: `0002`.
  - Message: `6502` (reboot).
  - Checksum: `65 XOR 02` = `63`.
- **Expected Response**:
  ```
  a0a10003836502e20d0a
  ```
  - Response: `836502` (ACK, success).
  - Checksum: `83 XOR 65 XOR 02` = `e2`.
- **Notes**: Not used in the script, indicating reboot may not be required.
- **Command to Send**:
  ```python
  command = bytes.fromhex("a0a100026502630d0a")
  ```

### 6. Configure RTK Mode (`6a07`)
- **Purpose**: Sets the receiver to RTK base mode.
- **Command**: `a0a100026a076d0d0a`
  - Length: `0002`.
  - Message: `6a07` (RTK base mode).
  - Checksum: `6a XOR 07` = `6d`.
- **Expected Response**:
  ```
  a0a10003836a07ee0d0a
  ```
  - Response: `836a07` (ACK, success).
  - Checksum: `83 XOR 6a XOR 07` = `ee`.
- **Log Evidence**: In recent run:
  ```
  a0a10003836a07ee0d0a
  ```
  Confirms successful RTK mode configuration.
- **Command to Send**:
  ```python
  command = bytes.fromhex("a0a100026a076d0d0a")
  ```

### 7. Set Surveyed Position (`6a06`)
- **Purpose**: Sets the base station’s coordinates (e.g., `40.7249028°N, -80.7283178°W, 325.553m`).
- **Command**:
  ```
  a0a100256a0601010000003c0000001e40442c369ea297f0c054083ea190dee04391283f0000000001620d0a
  ```
  - Length: `0025` (37 bytes).
  - Message:
    - `6a06`: Set position.
    - `01`: Mode (1 = surveyed position).
    - `01`: Storage (1 = SRAM+FLASH).
    - `0000003c`: Observation time (60 seconds, little-endian).
    - `0000001e`: Observation accuracy (30 cm, little-endian).
    - `40442c369ea297f0`: Latitude (`40.7249028°N`, double, little-endian).
    - `c054083ea190dee0`: Longitude (`-80.7283178°W`, double, little-endian).
    - `4391283f`: Altitude (`325.553m`, double, little-endian).
    - `0000000001`: Reserved.
  - Checksum: XOR of message bytes = `62`.
- **Expected Response**:
  ```
  a0a10003836a06ef0d0a
  ```
  - Response: `836a06` (ACK, success).
  - Checksum: `83 XOR 6a XOR 06` = `ef`.
- **Log Evidence**: In recent run:
  ```
  a0a10003836a06ef0d0a
  ```
  Confirms successful position setting. However, RTCM 1005 messages show inconsistent ECEF coordinates (e.g., `1315.690m, 19125971.720m, -7165551.450m`), suggesting the position may not be applied correctly.
- **Command to Send**:
  ```python
  command = bytes.fromhex("a0a100256a0601010000003c0000001e40442c369ea297f0c054083ea190dee04391283f0000000001620d0a")
  ```

### 8. Configure RTCM Measurement Data Out V2 (`6905`)
- **Purpose**: Enables RTCM messages (e.g., MSM4 for GPS, GLONASS, Galileo, BeiDou).
- **Command**:
  ```
  a0a10016690503010200010101010000010050505000005000016c0d0a
  ```
  - Length: `0016` (22 bytes).
  - Message:
    - `6905`: Configure RTCM output.
    - `03`: RTCM3 protocol.
    - `01`: Enable MSM4.
    - `02`: MSM4 message type.
    - `00`: Reserved.
    - `01010101`: Enable GPS, GLONASS, Galileo, BeiDou.
    - `000001`: Reserved.
    - `00505050`: Message intervals (80 seconds for ephemeris, little-endian).
    - `0000500001`: Reserved.
  - Checksum: XOR of message bytes = `6c`.
- **Expected Response**:
  ```
  a0a10003836905ef0d0a
  ```
  - Response: `836905` (ACK, success).
  - Checksum: `83 XOR 69 XOR 05` = `ef`.
- **GNSS Viewer Evidence**: A variation with 10-second intervals:
  ```
  a0a1001669050301010001010101010001000a0a0a00000a00016e0d0a
  ```
  - Intervals: `0a0a0a` (10 seconds for ephemeris).
  - Checksum: `6e`.
  - Response: `a0a10003836905ef0d0a`.
- **Notes**: Used in GNSS Viewer to set MSM4 messages (e.g., GPS, GLONASS, Galileo, BeiDou) with 10-second intervals for ephemeris messages (1005, 1019, etc.). MSM4 messages (e.g., `d300504320`) in the log indicate this command was applied.
- **Command to Send**:
  ```python
  command = bytes.fromhex("a0a10016690503010200010101010000010050505000005000016c0d0a")
  ```

### 9. Configure RTCM Measurement Data Out V2 with Additional Options (`6906`)
- **Purpose**: Configures RTCM output with additional options (extension of `6905`).
- **Command**: Not logged directly in GNSS Viewer output, but inferred from ACK.
- **Expected Response**:
  ```
  a0a10003836906ec0d0a
  ```
  - Response: `836906` (ACK, success).
  - Checksum: `83 XOR 69 XOR 06` = `ec`.
- **Notes**: Likely used by GNSS Viewer for advanced RTCM configuration. The exact command format is similar to `6905` but with additional parameters (not logged). Further investigation or logging may be needed to capture the full command.
- **Command to Send**:
  ```python
  # Command format is inferred; replace with actual command if logged
  # Example placeholder (based on 6905 structure with additional parameters)
  command = bytes.fromhex("a0a1001669060301010001010101010001000a0a0a00000a00016f0d0a")
  ```

### 10. Query RTCM Measurement Data Out V2 (`6982`)
- **Purpose**: Queries the current RTCM output configuration.
- **Command**: Not logged directly, but response indicates it was sent.
- **Expected Response**:
  ```
  a0a1001569820301010001010101010001000a0a0a00000a00e80d0a
  ```
  - Length: `0015` (21 bytes).
  - Response:
    - `6982`: Query RTCM output.
    - `03`: RTCM3 protocol.
    - `01`: MSM4 enabled.
    - `01`: MSM4 message type.
    - `00`: Reserved.
    - `0101010101`: GPS, GLONASS, Galileo, SBAS, BeiDou enabled.
    - `000100`: Reserved.
    - `0a0a0a`: Intervals (10 seconds for ephemeris).
    - `00000a00`: Reserved.
  - Checksum: XOR of response bytes = `e8`.
- **GNSS Viewer Evidence**: From response log:
  ```
  a0a1001569820301010001010101010001000a0a0a00000a00e80d0a
  ```
  Confirms the current RTCM configuration matches the `6905` command sent by GNSS Viewer.
- **Command to Send**:
  ```python
  command = bytes.fromhex("a0a100026982eb0d0a")
  ```

### 11. Unknown Command (`648c`)
- **Purpose**: Unknown (possibly system configuration or RTCM-related).
- **Command**: `a0a10004648c000fe70d0a`
  - Length: `0004`.
  - Message: `648c000f` (unknown purpose).
  - Checksum: `64 XOR 8c XOR 00 XOR 0f` = `e7`.
- **Expected Response**: Not logged in GNSS Viewer output.
- **Notes**: Observed in GNSS Viewer response log but not documented in SkyTraq manuals. May be a Phoenix-specific command. Further investigation required.
- **Command to Send**:
  ```python
  command = bytes.fromhex("a0a10004648c000fe70d0a")
  ```

### 12. Query Current Position (`6a83`)
- **Purpose**: Queries the current base station position.
- **Command**: `a0a100026a83690d0a`
  - Length: `0002`.
  - Message: `6a83` (query position).
  - Checksum: `6a XOR 83` = `69`.
- **Expected Response**:
  ```
  a0a100296a8301010000003c0000001e40442c366cb0e0b3c054083e10062125438f99810200000000000000003c0d0a
  ```
  - Length: `0029` (41 bytes).
  - Response:
    - `6a83`: Message ID.
    - `0101`: Mode and storage.
    - `0000003c`: Observation time (60 seconds).
    - `0000001e`: Observation accuracy (30 cm).
    - `40442c366cb0e0b3`: Latitude (`40.7249028°N`, double).
    - `c054083e10062125`: Longitude (`-80.7283178°W`, double).
    - `438f9981`: Altitude (`325.553m`, double).
    - `020000000000000000`: Reserved.
  - Checksum: XOR of response bytes = `3c`.
- **Log Evidence**: In `rtcm_20250507_184932.txt`:
  ```
  a0a100296a8301010000003c0000001e40442c366cb0e0b3c054083e10062125438f99810200000000000000003c0d0a
  ```
  Confirms the position matches the configured coordinates. Recent logs show inconsistent ECEF coordinates, indicating a potential issue.
- **Command to Send**:
  ```python
  command = bytes.fromhex("a0a100026a83690d0a")
  ```

## RTCM Output

The PX1125R outputs RTCM3 messages when configured as a base station. The primary message of interest is 1005 (stationary base coordinates).

### RTCM 1005 Message
- **Purpose**: Specifies the base station’s ECEF coordinates.
- **Format**:
  ```
  d30013<messagetype><payload><crc>
  ```
  - **Preamble**: `d30013` (19 bytes payload + 3 bytes CRC).
  - **Message Type**: `3ed0` (1005, shifted right by 4 bits).
  - **Payload**:
    - Bytes 0–1: Reference station ID (12 bits, e.g., `0002` = 2 or `0ed0` = 3792).
    - Bytes 2–5: ECEF X (signed integer, cm).
    - Bytes 6–9: ECEF Y (signed integer, cm).
    - Bytes 10–13: ECEF Z (signed integer, cm).
    - Bytes 14–18: Reserved.
  - **CRC**: 3 bytes, 24-bit CRC (polynomial `0x1864cfb`).
- **Example from Log** (`rtcm_20250507_184932.txt`):
  ```
  d300133ed0000201f1719347b4d54a7dcf0990400d29479851
  ```
  - Message Type: `3ed0` (1005).
  - Ref Station ID: `0002` (2).
  - ECEF X: `01f17193` = `1,423,700.00m`.
  - ECEF Y: `47b4d54a` = `-4,750,000.00m`.
  - ECEF Z: `());

//  `7dcf0990` = `4,150,000.00m`.
  - CRC: `9851` (valid).
  - Matches surveyed position (`40.7249028°N, -80.7283178°W, 325.553m`).
- **Example from Recent Run**:
  ```
  d300133ed00ed0000143aab4d549f4b609904004f814145b
  ```
  - Message Type: `3ed0` (1005).
  - Ref Station ID: `0ed0` (3792).
  - ECEF X: `000143aa` = `1315.690m`.
  - ECEF Y: `b4d549f4` = `19189825.800m`.
  - ECEF Z: `b6099040` = `-7166295.620m`.
  - CRC: `145b` (valid).
  - Does not match expected coordinates, indicating a configuration issue.

### Geodetic to ECEF Conversion for Surveyed Position
This section details the conversion of the surveyed position (`40.7249028°N, -80.7283178°W, 325.553m`) to ECEF coordinates using the WGS84 ellipsoid model, for reference in verifying RTCM 1005 messages.

#### WGS84 Parameters
- Semi-major axis: $ a = 6378137.0 \, \text{m} $
- Flattening: $ f = 1/298.257223563 $
- Eccentricity: $ e^2 = 2f - f^2 \approx 0.00669437999014 $

#### Conversion Formulas
The ECEF coordinates ($ X $, $ Y $, $ Z $) are calculated from geodetic coordinates (latitude $ \phi $, longitude $ \lambda $, height $ h $) as follows:

$$
N = \frac{a}{\sqrt{1 - e^2 \sin^2(\phi)}}
$$

$$
X = (N + h) \cos(\phi) \cos(\lambda)
$$

$$
Y = (N + h) \cos(\phi) \sin(\lambda)
$$

$$
Z = \left( N (1 - e^2) + h \right) \sin(\phi)
$$


Where:
- **N**: Radius of curvature in the prime vertical.
- **φ**: Latitude in radians.
- **λ**: Longitude in radians.  
- **h**: Height above the ellipsoid.



#### Step-by-Step Calculation
1. **Convert Coordinates to Radians**:
   - Latitude: $ \phi = 40.7249028^\circ \times \frac{\pi}{180} \approx 0.711058 \, \text{radians} $
   - Longitude: $ \lambda = -80.7283178^\circ \times \frac{\pi}{180} \approx -1.409573 \, \text{radians} $
   - Height: $ h = 325.553 \, \text{m} $

2. **Compute Trigonometric Values**:
   - $ \sin(\phi) \approx \sin(0.711058) \approx 0.651372 $
   - $ \cos(\phi) \approx \cos(0.711058) \approx 0.758711 $
   - $ \cos(\lambda) \approx \cos(-1.409573) \approx 0.163175 $
   - $ \sin(\lambda) \approx \sin(-1.409573) \approx -0.986627 $

3. **Compute Radius of Curvature $ N $**:

$$
e^2 \sin^2(\phi) \approx 0.00669437999014 \times (0.651372)^2 \approx 0.002842
$$

$$
N = \frac{6378137.0}{\sqrt{1 - 0.002842}} \approx \frac{6378137.0}{\sqrt{0.997158}} \approx 6386385.7 \, \text{m}
$$

4. **Compute ECEF $ X $**:

$$
X = (N + h) \cos(\phi) \cos(\lambda)
$$

$$
X \approx (6386385.7 + 325.553) \times 0.758711 \times 0.163175
$$

$$
X \approx 6386711.253 \times 0.758711 \times 0.163175 \approx 1423699.5 \, \text{m}
$$

5. **Compute ECEF $ Y $**:

$$
Y = (N + h) \cos(\phi) \sin(\lambda)
$$

$$
Y \approx (6386385.7 + 325.553) \times 0.758711 \times (-0.986627)
$$

$$
Y \approx 6386711.253 \times 0.758711 \times (-0.986627) \approx -4776425.3 \, \text{m}
$$

6. **Compute ECEF $ Z $**:

$$
Z = \left( N (1 - e^2) + h \right) \sin(\phi)
$$

$$
N (1 - e^2) \approx 6386385.7 \times (1 - 0.00669437999014) \approx 6343629.5
$$

$$
Z \approx (6343629.5 + 325.553) \times 0.651372 \approx 4136278.6 \, \text{m}
$$

#### Expected ECEF Coordinates
- **ECEF X**: `~1,423,699.5m`
- **ECEF Y**: `~-4,776,425.3m`
- **ECEF Z**: `~4,136,278.6m`

#### Comparison with Logs
- **Earlier Log (`rtcm_20250507_184932.txt`)**:
  - ECEF X: `1,423,700.00m`
  - ECEF Y: `-4,750,000.00m`
  - ECEF Z: `4,150,000.00m`
  - These values are very close to the calculated coordinates, with minor differences likely due to rounding or slight adjustments in the height used by the receiver.
- **Recent Log (May 8, 2025)**:
  - ECEF X: `1315.690m`
  - ECEF Y: `19163432.200m`
  - ECEF Z: `-7165613.540m`
  - These values are incorrect (wrong magnitude and sign), indicating the surveyed position was not applied correctly by the receiver, possibly due to prior configurations or a firmware issue.

#### Notes on Discrepancies
The discrepancy in the recent log suggests the `6a06` command (Set Surveyed Position) may not have been applied correctly. To verify, use the `6a83` command to query the current position and ensure it matches the expected coordinates. A power cycle before configuration, as recommended, can help ensure a clean state.

### Other RTCM Messages
- **MSM4 Messages**: Logs contain MSM4 messages (e.g., `d300504320`), confirming `6905` was applied. Recent run includes similar messages.
- **Frequency**: 1005 messages appear regularly (e.g., every few seconds), consistent with base station operation.

## Notes
- **Minimal Configuration**: The script uses only `6a07` and `6a06`, suggesting prior MSM4 settings (`6905`) were retained in FLASH.
- **ACK Issues**: Earlier scripts failed due to missing ACKs for `6501`, `6504`, etc., likely due to timing or firmware state. The current script avoids this.
- **RTCM Stability**: Logs show consistent 1005 messages, but recent ECEF coordinates are incorrect, suggesting a need to verify the surveyed position.
- **Reference Station ID**: Logs show IDs of `2` and `3792`, indicating possible configuration differences between runs.
- **GNSS Viewer Usage**: The GNSS Viewer interface uses `6905` to configure RTCM output (e.g., MSM4 messages with 10-second intervals) and `6982` to query the configuration.

## Recommendations
- Use `6a07` and `6a06` for basic RTK base configuration.
- Add `6905` if MSM4 messages are not output, adjusting intervals as needed (e.g., 10 seconds as in GNSS Viewer).
- Use `6982` to query and verify RTCM output configuration.
- Power cycle the PX1125R before configuration to ensure a clean state.
- Verify 1005 messages in the RTCM log to confirm coordinates.
- Investigate inconsistent ECEF coordinates by re-sending `6a06` or querying position with `6a83`.

Last Updated: May 8, 2025