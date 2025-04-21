import serial
import time
import sys
import termios
import tty

# Serial port settings
port_com1 = "/dev/ttyUSB_com1"  # Adjust to your COM1 port
port_com2 = "/dev/ttyUSB_com2"  # Adjust to your COM2 port
baudrate = 115200
timeout = 1

def clear_input_buffer():
    """Clear the input buffer to prevent leftover characters from affecting the next input."""
    # On Linux (Raspberry Pi), use termios to flush the input buffer
    try:
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
    except:
        # Fallback: Consume remaining input manually
        while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()

def send_command(command):
    """Send a command to COM1 and return the response."""
    try:
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

        print(f"\nConnected to {port_com1} at {baudrate} baud")
        print(f"Sent command: {command}")
        ser_com1.write((command + "\r\n").encode('ascii'))

        response = ""
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if ser_com1.in_waiting > 0:
                response += ser_com1.read(ser_com1.in_waiting).decode('ascii', errors='ignore')
            time.sleep(0.01)

        print("Response:")
        print(response)
        print(f"OK! Command accepted! Port: COM1.")  # Print only once

    except serial.SerialException as e:
        print(f"Serial error on COM1: {e}")
    finally:
        if 'ser_com1' in locals() and ser_com1.is_open:
            ser_com1.close()
            print("COM1 serial port closed")

def read_com2_messages():
    """Read and decode RTCM messages from COM2 for 15 seconds, extracting detailed info."""
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
        print("Reading messages from COM2 for 15 seconds...")

        buffer = bytearray()
        start_time = time.time()
        while (time.time() - start_time) < 15:
            if ser_com2.in_waiting > 0:
                data = ser_com2.read(ser_com2.in_waiting)
                buffer.extend(data)

                while len(buffer) >= 6:
                    if buffer[0] != 0xD3:
                        buffer = buffer[1:]
                        continue

                    length_bytes = buffer[1:3]
                    length = ((length_bytes[0] & 0x03) << 8) | length_bytes[1]

                    total_length = 1 + 2 + length + 3

                    if len(buffer) < total_length:
                        break

                    message_data = buffer[3:3 + length]

                    if len(message_data) >= 2:
                        message_id = ((message_data[0] << 4) | (message_data[1] >> 4)) & 0xFFF
                        print(f"Received RTCM {message_id} message")

                        # Decode RTCM 1006 (Station coordinates with antenna height, malformed)
                        if message_id == 1006:
                            print(f"RTCM 1006 - Raw Data (hex): {' '.join(f'{byte:02x}' for byte in message_data)}")
                            if len(message_data) >= 18:  # At least enough for ECEF X, Y, Z
                                station_id = (message_data[2] << 4) | (message_data[3] >> 4)
                                ecef_x = int.from_bytes(message_data[6:10], byteorder='big', signed=True) * 0.0001
                                ecef_y = int.from_bytes(message_data[10:14], byteorder='big', signed=True) * 0.0001
                                ecef_z = int.from_bytes(message_data[14:18], byteorder='big', signed=True) * 0.0001
                                print(f"RTCM 1006 - Station ID: {station_id}, ECEF X: {ecef_x:.4f} m, Y: {ecef_y:.4f} m, Z: {ecef_z:.4f} m")
                                if len(message_data) >= 23:  # Check for antenna height field
                                    ant_height = int.from_bytes(message_data[19:23], byteorder='big', signed=True) * 0.0001
                                    print(f"RTCM 1006 - Antenna Height: {ant_height:.4f} m")
                                else:
                                    print("RTCM 1006 - Antenna height field incomplete")
                            else:
                                print(f"RTCM 1006 - Message too short: {len(message_data)} bytes")

                        # Decode RTCM 1008 (Station ID and antenna serial number)
                        elif message_id == 1008:
                            if len(message_data) >= 5:
                                station_id = (message_data[2] << 4) | (message_data[3] >> 4)
                                ant_desc_len = message_data[4]
                                if 5 + ant_desc_len < len(message_data):
                                    ant_desc = message_data[5:5 + ant_desc_len].decode('ascii', errors='ignore')
                                    pos = 5 + ant_desc_len
                                    ant_sn_len = message_data[pos] if pos < len(message_data) else 0
                                    ant_sn = message_data[pos + 1:pos + 1 + ant_sn_len].decode('ascii', errors='ignore') if pos + 1 + ant_sn_len <= len(message_data) else ""
                                    print(f"RTCM 1008 - Station ID: {station_id}, Antenna Descriptor: {ant_desc}, Antenna Serial Number: {ant_sn}")
                                else:
                                    print(f"RTCM 1008 - Invalid antenna descriptor length: {ant_desc_len}")
                            else:
                                print(f"RTCM 1008 - Message too short: {len(message_data)} bytes")

                        # Decode RTCM 1033 (Receiver and antenna descriptors)
                        elif message_id == 1033:
                            if len(message_data) >= 5:
                                station_id = (message_data[2] << 4) | (message_data[3] >> 4)
                                print(f"RTCM 1033 - Raw Data (hex): {' '.join(f'{byte:02x}' for byte in message_data)}")
                                pos = 4
                                # Receiver descriptor (hardcode length due to firmware bug)
                                rcv_desc_len = 13  # "SINOGNSS K706"
                                if pos + 1 + rcv_desc_len <= len(message_data):
                                    rcv_desc = message_data[pos + 2:pos + 2 + rcv_desc_len].decode('ascii', errors='ignore')  # Skip 2 incorrect bytes
                                    pos += 3 + rcv_desc_len  # Skip 00 00 0d
                                else:
                                    rcv_desc = ""
                                    print(f"RTCM 1033 - Invalid receiver descriptor length: {rcv_desc_len}")
                                    continue
                                # Firmware version (hardcode length)
                                rcv_fw_len = 10  # "3.5.72.056"
                                if pos + 1 + rcv_fw_len <= len(message_data):
                                    # Replace null bytes with spaces to avoid truncation
                                    fw_data = bytearray(message_data[pos + 1:pos + 1 + rcv_fw_len])
                                    for i in range(len(fw_data)):
                                        if fw_data[i] == 0:
                                            fw_data[i] = ord(' ')
                                    rcv_fw = fw_data.decode('ascii', errors='ignore')
                                    pos += 1 + rcv_fw_len
                                else:
                                    rcv_fw = ""
                                    print(f"RTCM 1033 - Invalid firmware length: {rcv_fw_len}")
                                    continue
                                # Receiver serial number (hardcode length)
                                rcv_sn_len = 8  # "02605171"
                                if pos + 1 + rcv_sn_len <= len(message_data):
                                    rcv_sn = message_data[pos + 1:pos + 1 + rcv_sn_len].decode('ascii', errors='ignore')
                                    pos += 1 + rcv_sn_len
                                else:
                                    rcv_sn = ""
                                    print(f"RTCM 1033 - Invalid receiver serial length: {rcv_sn_len}")
                                    continue
                                # Antenna descriptor (should be empty)
                                ant_desc_len = 0
                                ant_desc = ""
                                pos += 1
                                # Antenna serial number (should be empty)
                                ant_sn_len = 0
                                ant_sn = ""
                                pos += 1
                                # Fixed print statement to include station ID
                                print(f"RTCM 1033 - Station ID: {station_id}, Receiver Descriptor: {rcv_desc}, Firmware: {rcv_fw}, Receiver Serial: {rcv_sn}, Antenna Descriptor: {ant_desc}, Antenna Serial Number: {ant_sn}")
                            else:
                                print(f"RTCM 1033 - Message too short: {len(message_data)} bytes")

                    buffer = buffer[total_length:]

            time.sleep(0.1)

    except serial.SerialException as e:
        print(f"Serial error on COM2: {e}")
    finally:
        if 'ser_com2' in locals() and ser_com2.is_open:
            ser_com2.close()
            print("COM2 serial port closed")

def reconfigure_rtk_base():
    """Reconfigure the K706 as an RTK base station with RTCM output."""
    commands = [
        "UNLOGALL COM2",  # Stop all logs on COM2
        "FIX POSITION 40.34536010088 -80.12878619119 326.5974",  # Set fixed position
        "LOG COM2 RTCM1006B ONTIME 10",  # Station coordinates
        "LOG COM2 RTCM1008B ONTIME 5",   # Station ID and antenna info
        "LOG COM2 RTCM1033B ONTIME 10",  # Receiver/antenna descriptors
        "LOG COM2 RTCM1004B ONTIME 1",   # GPS observations
        "LOG COM2 RTCM1012B ONTIME 1",   # GLONASS observations
        "LOG COM2 RTCM1094B ONTIME 1",   # Galileo observations (if supported)
        "LOG COM2 RTCM1124B ONTIME 1",   # BeiDou observations
        "SAVECONFIG"  # Save configuration
    ]
    for cmd in commands:
        send_command(cmd)

def print_menu():
    """Print the menu options."""
    print("\n1. Send LOG VERSION")
    print("2. Reconfigure as RTK Base Station (restore RTCM output)")
    print("3. Send LOG COMCONFIG (show port configurations)")
    print("4. Print 15 seconds of messages from COM2")
    print("5. Send FIX POSITION (set fixed position for RTK base)")
    print("6. Send Custom Command")
    print("7. Exit")

def main():
    while True:
        print_menu()
        # Clear the input buffer before prompting for the menu choice
        clear_input_buffer()
        choice = input("\nEnter your choice (1-7): ").strip()

        if choice == "1":
            send_command("LOG VERSION ONCE")
        elif choice == "2":
            reconfigure_rtk_base()
        elif choice == "3":
            send_command("LOG COMCONFIG ONCE")
        elif choice == "4":
            read_com2_messages()
        elif choice == "5":
            send_command("FIX POSITION 40.34536010088 -80.12878619119 326.5974")
        elif choice == "6":
            custom_command = input("Enter custom command: ").strip()
            send_command(custom_command)
            # Clear the input buffer after entering the custom command
            clear_input_buffer()
        elif choice == "7":
            print("Exiting...")
            break
        else:
            print("Invalid choice, please select 1-7")

if __name__ == "__main__":
    main()