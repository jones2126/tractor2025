import serial
import time

# Serial port settings
port = "/dev/ttyUSB0"  # COM1 on your Raspberry Pi
baudrate = 115200
timeout = 1  # Timeout for reading (in seconds)

try:
    # Open the serial port
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
        xonxoff=False,  # Disable software flow control
        rtscts=False,   # Disable hardware flow control (RTS/CTS)
        dsrdtr=False    # Disable hardware flow control (DSR/DTR)
    )

    print(f"Connected to {port} at {baudrate} baud")

    # Ensure the serial port is open
    if ser.is_open:
        # Send UNLOGALL to clear any active logs
        command = "UNLOGALL\r\n"
        ser.write(command.encode('ascii'))
        print(f"Sent command: {command.strip()}")

        # Read the response to UNLOGALL (up to 2 seconds)
        print("Reading response to UNLOGALL...")
        start_time = time.time()
        while (time.time() - start_time) < 2:
            if ser.in_waiting > 0:
                response = ser.readline().decode('ascii', errors='ignore').strip()
                if response:
                    print(f"Response: {response}")
            time.sleep(0.1)

        # Send the LOG VERSION command
        command = "LOG VERSION\r\n"
        ser.write(command.encode('ascii'))
        print(f"\nSent command: {command.strip()}")

        # Read the response to LOG VERSION (up to 5 seconds)
        print("Reading response to LOG VERSION...")
        start_time = time.time()
        while (time.time() - start_time) < 5:
            if ser.in_waiting > 0:
                response = ser.readline().decode('ascii', errors='ignore').strip()
                if response:
                    print(f"Response: {response}")
            time.sleep(0.1)

        if time.time() - start_time >= 5 and not ser.in_waiting:
            print("No further response received")

    else:
        print("Failed to open serial port")

except serial.SerialException as e:
    print(f"Serial error: {e}")
except Exception as e:
    print(f"Error: {e}")
finally:
    # Close the serial port
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed")
