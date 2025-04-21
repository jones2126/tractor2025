import serial
import time

# Serial port settings for COM1
port = "/dev/ttyUSB_com1"  # COM1 (fixed symbolic link)
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
    "LOG COM2 RTCM1114B ONTIME 1",  # QZSS L1/L2 observations
    "LOG COM2 RTCM1006B ONTIME 10", # Station coordinates
    "LOG COM2 RTCM1008B ONTIME 5",  # Station ID and antenna info
    "LOG COM2 RTCM1033B ONTIME 10", # Receiver and antenna descriptors
    "LOG COM2 RTCM1019B ONTIME 10", # GPS ephemeris
    "LOG COM2 RTCM1020B ONTIME 10", # GLONASS ephemeris
    "LOG COM2 RTCM1045B ONTIME 10", # Galileo ephemeris
    "LOG COM2 RTCM1042B ONTIME 10", # BeiDou ephemeris
    "LOG COM2 RTCM1044B ONTIME 10", # QZSS ephemeris
    "SAVECONFIG"
]

def send_command(ser, command):
    """Send a command to the K706 and return the response."""
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
    print("K706 Command Menu\n")

    while True:
        # Display the menu
        print("1. Send LOG VERSION")
        print("2. Reconfigure as RTK Base Station (restore RTCM output)")
        print("3. Send LOG COMCONFIG (show port configurations)")
        print("4. Send LOG BESTPOSA ONCE (show current position and fix status)")
        print("5. Send FIX POSITION (set fixed position for RTK base)")
        print("6. Send Custom Command")
        print("7. Exit")
        choice = input("\nEnter your choice (1-7): ")

        if choice == "1":
            send_command(ser, "LOG VERSION")

        elif choice == "2":
            print("\nReconfiguring K706 as RTK Base Station...")
            for command in rtcm_config_commands:
                send_command(ser, command)
                time.sleep(0.5)  # Small delay between commands

        elif choice == "3":
            send_command(ser, "LOG COMCONFIG")

        elif choice == "4":
            send_command(ser, "LOG BESTPOSA ONCE")

        elif choice == "5":
            print("\nSetting fixed position for RTK base...")
            send_command(ser, "FIX POSITION 40.34536010088 -80.12878619119 326.5974")

        elif choice == "6":
            custom_command = input("Enter custom command: ")
            send_command(ser, custom_command)

        elif choice == "7":
            print("Exiting...")
            break

        else:
            print("Invalid choice, please select 1-7")

except serial.SerialException as e:
    print(f"Serial error: {e}")
except Exception as e:
    print(f"Error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed")