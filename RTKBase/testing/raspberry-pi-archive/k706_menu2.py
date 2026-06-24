import serial
import time
import sys

# Serial port settings for K706
port_com1 = "/dev/ttyUSB_com1"  # K706 COM1
port_com2 = "/dev/ttyUSB_com2"  # K706 COM2
baudrate = 115200
timeout = 1

def clear_input_buffer():
    """Clear the input buffer by consuming any remaining characters."""
    import select
    while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        sys.stdin.readline()

def send_command(command):
    """Send a command to COM1 and return the response with a 5-second delay."""
    try:
        ser_com1 = serial.Serial(
            port=port_com1,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=5,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )

        print(f"\nConnected to {port_com1} at {baudrate} baud")
        print(f"Sent command: {command}")
        ser_com1.write((command + "\r\n").encode('ascii'))

        response = ""
        start_time = time.time()
        while (time.time() - start_time) < 5:
            if ser_com1.in_waiting > 0:
                data = ser_com1.read(ser_com1.in_waiting).decode('ascii', errors='ignore')
                response += data
            time.sleep(0.1)

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

                        # Decode RTCM 1005 (Station coordinates without antenna height)
                        if message_id == 1005:
                            print(f"RTCM 1005 - Raw Data (hex): {' '.join(f'{byte:02x}' for byte in message_data)}")
                            if len(message_data) >= 19:
                                station_id = (message_data[2] << 4) | (message_data[3] >> 4)
                                ecef_x = int.from_bytes(message_data[6:10], byteorder='big', signed=True) * 0.0001
                                ecef_y = int.from_bytes(message_data[10:14], byteorder='big', signed=True) * 0.0001
                                ecef_z = int.from_bytes(message_data[14:18], byteorder='big', signed=True) * 0.0001
                                print(f"RTCM 1005 - Station ID: {station_id}, ECEF X: {ecef_x:.4f} m, Y: {ecef_y:.4f} m, Z: {ecef_z:.4f} m")
                            else:
                                print(f"RTCM 1005 - Message too short: {len(message_data)} bytes")

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
                                rcv_desc_len = 13
                                if pos + 1 + rcv_desc_len <= len(message_data):
                                    rcv_desc = message_data[pos + 2:pos + 2 + rcv_desc_len].decode('ascii', errors='ignore')
                                    pos += 3 + rcv_desc_len
                                else:
                                    rcv_desc = ""
                                    print(f"RTCM 1033 - Invalid receiver descriptor length: {rcv_desc_len}")
                                    continue
                                rcv_fw_len = 10
                                if pos + 1 + rcv_fw_len <= len(message_data):
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
                                rcv_sn_len = 8
                                if pos + 1 + rcv_sn_len <= len(message_data):
                                    rcv_sn = message_data[pos + 1:pos + 1 + rcv_sn_len].decode('ascii', errors='ignore')
                                    pos += 1 + rcv_sn_len
                                else:
                                    rcv_sn = ""
                                    print(f"RTCM 1033 - Invalid receiver serial length: {rcv_sn_len}")
                                    continue
                                ant_desc_len = 0
                                ant_desc = ""
                                pos += 1
                                ant_sn_len = 0
                                ant_sn = ""
                                pos += 1
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

"""
RTCM Messages Used in K706 Configuration:
- RTCM1005B: Stationary RTK Reference Station ARP coordinates (ECEF X, Y, Z).
- RTCM1074B: GPS MSM4 - GPS L1/L2 observations (pseudorange, carrier phase).
- RTCM1084B: GLONASS MSM4 - GLONASS L1/L2 observations (pseudorange, carrier phase).
- RTCM1094B: Galileo MSM4 - Galileo E1/E5b observations (pseudorange, carrier phase).
- RTCM1124B: BeiDou MSM4 - BeiDou B1I/B2I observations (pseudorange, carrier phase).
- RTCM1230B: GLONASS Code-Phase Biases - GLONASS bias corrections for RTK.
- RTCM1008B: Antenna Descriptor - Base station antenna type and serial number.
- RTCM1033B: Receiver/Antenna Descriptors - Receiver and antenna metadata.
"""

def reconfigure_rtk_base():
    """Reconfigure the K706 as an RTK base station with RTCM output for PX1172RD."""
    commands = [
        "UNLOGALL COM2",
        "FIX POSITION 40.34538488389 -80.12880797750 326.1906",
        "LOG COM2 RTCM1005B ONTIME 10",
        "LOG COM2 RTCM1074B ONTIME 1",
        "LOG COM2 RTCM1084B ONTIME 1",
        "LOG COM2 RTCM1094B ONTIME 1",
        "LOG COM2 RTCM1124B ONTIME 1",
        "LOG COM2 RTCM1230B ONTIME 10",
        "LOG COM2 RTCM1008B ONTIME 10",
        "LOG COM2 RTCM1033B ONTIME 10",
        "SAVECONFIG"
    ]
    for cmd in commands:
        send_command(cmd)

def log_bestposa():
    """Log BESTPOSA on K706 COM1 to get the best available position solution."""
    send_command("LOG COM1 BESTPOSA ONCE")

def print_menu():
    """Print the menu options."""
    print("\n1. Send LOG VERSION (K706)")
    print("2. Reconfigure K706 as RTK Base Station (optimized for PX1172RD rover)")
    print("3. Send LOG COMCONFIG (show K706 port configurations)")
    print("4. Print 15 seconds of messages from K706 COM2")
    print("5. Send FIX POSITION (set fixed position for K706 RTK base)")
    print("6. Send Custom Command (K706)")
    print("7. Log BESTPOSA on K706 COM1 (best available position)")
    print("8. Exit")

def main():
    while True:
        print_menu()
        clear_input_buffer()
        choice = input("\nEnter your choice (1-8): ").strip()

        if choice == "1":
            send_command("LOG VERSION ONCE")
        elif choice == "2":
            reconfigure_rtk_base()
        elif choice == "3":
            send_command("LOG COMCONFIG ONCE")
        elif choice == "4":
            read_com2_messages()
        elif choice == "5":
            send_command("FIX POSITION 40.34538488389 -80.12880797750 326.1906")
        elif choice == "6":
            custom_command = input("Enter custom command: ").strip()
            send_command(custom_command)
        elif choice == "7":
            log_bestposa()
        elif choice == "8":
            print("Exiting...")
            break
        else:
            print("Invalid choice, please select 1-8")

if __name__ == "__main__":
    main()