from pymodbus.client.serial import ModbusSerialClient

# Create Modbus client with correct serial settings
client = ModbusSerialClient(
    port="/dev/ttyUSB0",  # your FTDI adapter
    baudrate=9600,
    parity='N',
    stopbits=1,
    bytesize=8,
    timeout=2
)

# Connect to the controller
if client.connect():
    print("Connected to Renogy controller.")

    try:
        # Use slave ID 255
        response = client.read_holding_registers(address=0, count=1, slave=255)

        if response.isError():
            print(f"Modbus Error: {response}")
        else:
            print(f"Registers: {response.registers}")
    except Exception as e:
        print(f"Exception: {e}")
    finally:
        client.close()
else:
    print("Failed to connect to controller.")
