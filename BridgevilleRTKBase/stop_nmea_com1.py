import serial
import time

port = "/dev/ttyUSB0"
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
    if ser.is_open:
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

    ser.close()
    print("Serial port closed")

except serial.SerialException as e:
    print(f"Serial error: {e}")
except Exception as e:
    print(f"Error: {e}")
