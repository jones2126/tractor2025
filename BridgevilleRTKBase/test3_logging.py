from pymodbus.client.serial import ModbusSerialClient

# Setup serial connection
client = ModbusSerialClient(
    port="/dev/ttyUSB0",  # Adjust if needed
    baudrate=9600,
    parity='N',
    stopbits=1,
    bytesize=8,
    timeout=1
)

print("Scanning Modbus slave IDs...")
if client.connect():
    for slave_id in range(1, 256):
        print(f"Trying ID {slave_id}...", end="")
        try:
            response = client.read_holding_registers(address=0, count=1, slave=slave_id)
            if not response.isError():
                print(f" SUCCESS — Got response: {response.registers}")
                break
            else:
                print(" no response.")
        except Exception as e:
            print(f" error: {e}")
    client.close()
else:
    print("Failed to open serial connection.")
