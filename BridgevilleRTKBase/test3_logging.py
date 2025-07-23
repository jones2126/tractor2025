import serial

# Open the FTDI serial port at 9600 baud
ser = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=1)

# Send a raw Modbus RTU query to read the first input register (battery voltage)
# Format: [slave_id, function_code, address_hi, address_lo, count_hi, count_lo, CRC_lo, CRC_hi]
query = bytes.fromhex("010300000001840A")

print("Sending query to Renogy controller...")
ser.write(query)

# Read up to 16 bytes of response
response = ser.read(16)
print("Response (raw bytes):", response)
print("Response (hex):", response.hex())
