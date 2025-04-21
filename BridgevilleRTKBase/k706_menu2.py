import serial
import time

# Serial port settings
port_com1 = "/dev/ttyUSB_com1"  # Adjust to your COM1 port
port_com2 = "/dev/ttyUSB_com2"  # Adjust to your COM2 port
baudrate = 115200
timeout = 1

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
        print(f"OK! Command accepted! Port: COM1.")

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
        while (time.time() - start_time) < 15:  # Capture for 15 seconds
            if ser_com2.in_waiting > 0:
                data = ser_com2.read(ser_com2.in_waiting)
                buffer.extend(data)

                while len(buffer) >= 6:
                    # Check for RTCM preamble (0xD3)
                    if buffer[0] != 0xD3:
                        buffer = buffer[1:]
                        continue

                    # Extract message length
                    length_bytes = buffer[1:3]
                    length = ((length_bytes[0] & 0x03) << 8) | length_bytes[1]

                    total_length = 1 + 2 + length + 3  # Preamble + Length + Data + CRC

                    if len(buffer) < total_length:
                        break

                    message_data = buffer[3:3 + length]

                    if len(message_data) >= 2:
                        # Extract message ID
                        message_id = ((message_data[0] << 4) | (message_data[1] >> 4)) & 0xFFF
                        print(f"Received RTCM {message_id} message")

                        # Decode specific RTCM messages
                        if message_id == 1006:  # Station coordinates with antenna height
                            if len(message_data) >= 26:
                                station_id = (message_data[2] << 4) | (message_data[3] >> 4)
                                ecef_x = int.from_bytes(message_data[6:10], byteorder='big', signed=True) * 0.0001
                                ecef_y = int.from_bytes(message_data[10:14], byteorder='big', signed=True) * 0.0001
                                ecef_z = int.from_bytes(message_data[14:18], byteorder='big', signed=True) * 0.0001
                                ant_height = int.from_bytes(message_data[22:26], byteorder='big', signed=True) * 0.0001
                                print(f"RTCM 1006 - Station ID: {station_id}, ECEF X: {ecef_x:.4f} m, Y: {ecef_y:.4f} m, Z: {ecef_z:.4f} m, Antenna Height: {ant_height:.4f} m")

                        elif message_id == 1008:  # Station ID and antenna serial number
                            if len(message_data) >= 6:
                                station_id = (message_data[2] << 4) | (message_data[3] >> 4)
                                ant_desc_len = message_data[4]
                                ant_desc = message_data[5:5 + ant_desc_len].decode('ascii', errors='ignore')
                                pos = 5 + ant_desc_len
                                ant_sn_len = message_data[pos] if pos < len(message_data) else 0
                                ant_sn = message_data[pos + 1:pos + 1 + ant_sn_len].decode('ascii', errors='ignore') if pos + 1 + ant_sn_len <= len(message_data) else ""
                                print(f"RTCM 1008 - Station ID: {station_id}, Antenna Descriptor: {ant_desc}, Antenna Serial Number: {ant_sn}")

                        elif message_id == 1033:  # Receiver and antenna descriptors
                            if len(message_data) >= 8:
                                station_id = (message_data[2] << 4) | (message_data[3] >> 4)
                                rcv_desc_len = message_data[4]
                                rcv_desc = message_data[5:5 + rcv_desc_len].decode('ascii', errors='ignore')
                                pos = 5 + rcv_desc_len
                                rcv_fw_len = message_data[pos] if pos < len(message_data) else 0
                                rcv_fw = message_data[pos + 1:pos + 1 + rcv_fw_len].decode('ascii', errors='ignore') if pos + 1 + rcv_fw_len <= len(message_data) else ""
                                pos += 1 + rcv_fw_len
                                rcv_sn_len = message_data[pos] if pos < len(message_data) else 0
                                rcv_sn = message_data[pos + 1:pos + 1 + rcv_sn_len].decode('ascii', errors='ignore') if pos + 1 + rcv_sn_len <= len(message_data) else ""
                                pos += 1 + rcv_sn_len
                                ant_desc_len = message_data[pos] if pos < len(message_data) else 0
                                ant_desc = message_data[pos + 1:pos + 1 + ant_desc_len].decode('ascii', errors='ignore') if pos + 1 + ant_desc_len <= len(message_data) else ""
                                pos += 1 + ant_desc_len
                                ant_sn_len = message_data[pos] if pos < len(message_data) else 0
                                ant_sn = message_data[pos + 1:pos + 1 + ant_sn_len].decode('ascii', errors='ignore') if pos + 1 + ant_sn_len <= len(message_data) else ""
                                print(f"RTCM 1033 - Station ID: {station_id}, Receiver Descriptor: {rcv_desc}, Firmware: {rcv_fw}, Receiver Serial: {rcv_sn}, Antenna Descriptor: {ant_desc}, Antenna Serial Number: {ant_sn}")

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
        choice = input("\nEnter your choice (1-7): ")

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
            custom_command = input("Enter custom command: ")
            send_command(custom_command)
        elif choice == "7":
            print("Exiting...")
            break
        else:
            print("Invalid choice, please select 1-7")

if __name__ == "__main__":
    main()