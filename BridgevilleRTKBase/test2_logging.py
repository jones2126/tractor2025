import serial
import time

# Serial port settings
port = "/dev/ttyUSB0"  # COM1 on your Raspberry Pi
baudrate = 115200
timeout = 1  # Timeout for reading (in seconds)

# List of line endings to test
line_endings = [
    "\r\n",  # CR+LF (carriage return + line feed, standard for many serial protocols)
    "\r",    # CR only (carriage return)
    "\n",    # LF only (line feed)
    ""       # No line ending
]

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
        # Clear any active logs to avoid interference
        ser.write("UNLOGALL\r\n".encode('ascii'))
        time.sleep(0.1)

        # Test each line ending
        for ending in line_endings:
            # Send the LOG VERSION command with the current line ending
            command = f"LOG VERSION{ending}"
            ser.write(command.encode('ascii'))
            print(f"Sent command with ending '{ending}': {command.strip()}")

            # Wait for a response (up to 5 seconds)
            start_time = time.time()
            response_received = False
            while (time.time() - start_time) < 5:
                if ser.in_waiting > 0:
                    response = ser.readline().decode('ascii', errors='ignore').strip()
                    if response:
                        print(f"Response: {response}")
                        response_received = True
                        break
                time.sleep(0.1)

            if not response_received:
                print("No response received")
            print("---")

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
