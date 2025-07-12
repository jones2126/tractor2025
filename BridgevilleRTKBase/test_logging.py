import serial
import time

# Configure the serial port (adjust '/dev/ttyUSB0' to match your RPi's port)
ser = serial.Serial(
    port='/dev/tttyACM0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

print("Listening for GPS data on /dev/ttyUSB0...")

try:
    while True:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)  # Read available data
            for i in range(len(data)):
                # Check for message headers
                if data[i:i+3] == b'\xB5\x62':  # UBX header (0xB5 0x62)
                    print(f"UBX Message detected at offset {i}")
                elif data[i:i+1] == b'$':  # NMEA header
                    print(f"NMEA Message detected at offset {i}")
                elif data[i:i+1] == b'\xD3':  # RTCM3 header
                    print(f"RTCM3 Message detected at offset {i}")
            time.sleep(0.1)  # Small delay to prevent overwhelming the CPU

except KeyboardInterrupt:
    print("Stopped by user")
    ser.close()

except serial.SerialException as e:
    print(f"Serial error: {e}")
    ser.close()