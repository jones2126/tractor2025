import serial
import time

# Serial port settings for COM2
port = "/dev/ttyUSB_com2"  # COM2 (fixed symbolic link)
baudrate = 115200
timeout = 1

try:
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False
    )

    print(f"Connected to {port} at {baudrate} baud")
    print("Reading RTCM corrections from COM2... (press Ctrl+C to stop)")

    try:
        while True:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                hex_data = data.hex()
                print(f"Raw RTCM (hex): {hex_data}")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopped by user")

except serial.SerialException as e:
    print(f"Serial error: {e}")
except Exception as e:
    print(f"Error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed")