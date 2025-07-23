from pymodbus.client.serial import ModbusSerialClient

# Create the RTU client (no 'method' argument in pymodbus ≥ 3.x)
client = ModbusSerialClient(
    port="/dev/ttyUSB0",
    baudrate=9600,
    stopbits=1,
    bytesize=8,
    parity="N",
    timeout=1
)

# Connect
if not client.connect():
    print("Failed to connect to /dev/ttyUSB0")
    exit(1)

print("Connected to Renogy controller.")

# Attempt to read 1 holding register starting at address 0
response = client.read_holding_registers(address=0, count=1, unit=1)

if response.isError():
    print("Modbus error:", response)
else:
    value = response.registers[0]
    print("Raw value:", value)
    print("Battery voltage:", value / 100.0, "V")

client.close()
