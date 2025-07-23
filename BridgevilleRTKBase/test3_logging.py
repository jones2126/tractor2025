from pymodbus.client.serial import ModbusSerialClient as ModbusClient

# Setup Modbus RTU client on /dev/ttyUSB0 at 9600 baud
client = ModbusClient(
    method='rtu',
    port='/dev/ttyUSB0',
    baudrate=9600,
    stopbits=1,
    bytesize=8,
    parity='N',
    timeout=1
)

# Connect to the serial port
if not client.connect():
    print("Failed to connect to /dev/ttyUSB0")
    exit(1)

print("Connected to Renogy controller.")

# Try reading 1 register starting at address 0 (battery voltage, often)
response = client.read_holding_registers(address=0, count=1, unit=1)

if response.isError():
    print("Modbus error:", response)
else:
    value = response.registers[0]
    print("Raw value:", value)
    print("Interpreted battery voltage:", value / 100.0, "V")

client.close()
