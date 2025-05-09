import serial
import time

# Serial port settings for COM1 (commands)
port_com1 = "/dev/ttyUSB_com1"  # COM1 (fixed symbolic link)
# Serial port settings for COM2 (RTCM output)
port_com2 = "/dev/ttyUSB_com2"  # COM2 (fixed symbolic link)
baudrate = 115200
timeout = 1  # Timeout for reading (in seconds)

# RTCM configuration commands
rtcm_config_commands = [
    "UNLOGALL",
    "FIX POSITION 40.34536010088 -80.12878619119 326.5974",
    "LOG COM2 RTCM1004B ONTIME 1",  # GPS L1/L2 observations
    "LOG COM2 RTCM1012B ONTIME 1",  # GLONASS L1/L2 observations
    "LOG COM2 RTCM1094B ONTIME 1",  # Galileo E1/E5b observations
    "LOG COM2 RTCM1124B ONTIME 1",  # BeiDou B1/B2 observations
    "LOG COM2 RTCM1006B ONTIME 10", # Station coordinates
    "LOG COM2 RTCM1008B ONTIME 5",  # Station ID and antenna info
    "LOG COM2 RTCM1033B ONTIME 10", # Receiver/antenna descriptors
    "LOG COM2 RTCM1019B ONTIME 10", # GPS ephemeris
    "LOG COM2 RTCM1020B ONTIME 10", # GLONASS ephemeris
    "LOG COM2 RTCM1045B ONTIME 10", # Galileo ephemeris
    "LOG COM2 RTCM1042B ONTIME 10", # BeiDou ephemeris
    "SAVECONFIG"
]

def send_command(ser, command):
    """Send a command to the K706 via COM1 and return the response."""
    ser.write((command + "\r\n").encode('ascii'))
    print(f"\nSent command: {command}")

    # Wait for the response
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

def read_com2_messages():
    """Read and identify RTCM messages from COM2 for 5 seconds, printing message types in ASCII."""
    try:
        ser_com2 = serial.Serial(
            port=port_com2,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )

        print(f"\nConnected to {port_com2} at {baudrate} baud")
        print("Reading messages from COM2 for 5 seconds...")

        buffer = bytearray()
        start_time = time.time()
        while (time.time() - start_time) < 5:
            if ser_com2.in_waiting > 0:
                # Read available data and append to buffer
                data = ser_com2.read(ser_com2.in_waiting)
                buffer.extend(data)

                # Process the buffer for complete RTCM messages
                while len(buffer) >= 6:  # Minimum length for preamble, length field, and some data
                    # Check for RTCM preamble (0xD3)
                    if buffer[0] != 0xD3:
                        # Skip bytes until we find the preamble
                        buffer = buffer[1:]
                        continue

                    # Extract the message length (10 bits, after 6 reserved bits)
                    length_bytes = buffer[1:3]  # Bytes 1 and 2 (after preamble)
                    length = ((length_bytes[0] & 0x03) << 8) | length_bytes[1]  # Last 2 bits of byte 1 + byte 2

                    # Total message length: preamble (1 byte) + length field (2 bytes) + data (length bytes) + CRC (3 bytes)
                    total_length = 1 + 2 + length + 3

                    # Check if we have the full message
                    if len(buffer) < total_length:
                        break  # Wait for more data

                    # Extract the message data
                    message_data = buffer[3:3 + length]

                    # Extract the message ID (first 12 bits of the message data)
                    if len(message_data) >= 2:
                        message_id = ((message_data[0] << 4) | (message_data[1] >> 4)) & 0xFFF  # 12-bit message ID
                        print(f"Received RTCM {message_id} message")

                    # Remove the processed message from the buffer
                    buffer = buffer[total_length:]

            time.sleep(0.1)

    except serial.SerialException as e:
        print(f"Serial error on COM2: {e}")
    finally:
        if 'ser_com2' in locals() and ser_com2.is_open:
            ser_com2.close()
            print("COM2 serial port closed")

try:
    # Open the serial port for COM1
    ser_com1 = serial.Serial(
        port=port_com1,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False
    )

    print(f"Connected to {port_com1} at {baudrate} baud")
    print("K706 Command Menu\n")

    while True:
        # Display the menu
        print("1. Send LOG VERSION")
        print("2. Reconfigure as RTK Base Station (restore RTCM output)")
        print("3. Send LOG COMCONFIG (show port configurations)")
        print("4. Print 5 seconds of messages from COM2")
        print("5. Send FIX POSITION (set fixed position for RTK base)")
        print("6. Send Custom Command")
        print("7. Exit")
        choice = input("\nEnter your choice (1-7): ")

        if choice == "1":
            send_command(ser_com1, "LOG VERSION")

        elif choice == "2":
            print("\nReconfiguring K706 as RTK Base Station...")
            for command in rtcm_config_commands:
                send_command(ser_com1, command)
                time.sleep(0.5)  # Small delay between commands

        elif choice == "3":
            send_command(ser_com1, "LOG COMCONFIG")

        elif choice == "4":
            read_com2_messages()

        elif choice == "5":
            print("\nSetting fixed position for RTK base...")
            send_command(ser_com1, "FIX POSITION 40.34536010088 -80.12878619119 326.5974")

        elif choice == "6":
            custom_command = input("Enter custom command: ")
            send_command(ser_com1, custom_command)

        elif choice == "7":
            print("Exiting...")
            break

        else:
            print("Invalid choice, please select 1-7")

except serial.SerialException as e:
    print(f"Serial error on COM1: {e}")
except Exception as e:
    print(f"Error: {e}")
finally:
    if 'ser_com1' in locals() and ser_com1.is_open:
        ser_com1.close()
        print("COM1 serial port closed")