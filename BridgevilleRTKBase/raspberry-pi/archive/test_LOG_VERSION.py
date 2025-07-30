import serial
import time

# Serial port settings
baudrate = 115200
timeout = 1  # Timeout for reading (in seconds)
ports = ["/dev/ttyUSB_com1", "/dev/ttyUSB_com2"]  # Test both ports

for port in ports:
    print(f"\nTesting port: {port}")
    try:
        # Open the serial port
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

        # Ensure the serial port is open
        if ser.is_open:
            # Send the LOG VERSION command
            command = "LOG VERSION\r\n"
            ser.write(command.encode('ascii'))
            print(f"Sent command: {command.strip()}")

            # Wait for the response (increase wait time to 1 second)
            time.sleep(1)

            # Read the response (read multiple lines if necessary)
            response_lines = []
            start_time = time.time()
            while (time.time() - start_time) < 2:  # Wait up to 2 seconds
                if ser.in_waiting > 0:
                    response = ser.readline().decode('ascii', errors='ignore').strip()
                    if response:
                        response_lines.append(response)
                time.sleep(0.1)

            if response_lines:
                print("Response:")
                for line in response_lines:
                    print(line)
            else:
                print("No response received")

        else:
            print("Failed to open serial port")

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed")