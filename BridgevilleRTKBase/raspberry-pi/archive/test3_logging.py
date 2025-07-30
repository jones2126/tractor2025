from pymodbus.client.serial import ModbusSerialClient

client = ModbusSerialClient(
    port='/dev/ttyUSB0',
    baudrate=9600,
    stopbits=1,
    bytesize=8,
    parity='N',
    timeout=1
)

if client.connect():
    print("Connected to Renogy controller.")
    result = client.read_holding_registers(address=0x100, count=35, slave=255)
    if result.isError():
        print(f"Modbus error: {result}")
    else:
        registers = result.registers
        battery_soc = registers[0]
        battery_voltage = registers[1] * 0.1
        battery_charging_amps = registers[2] * 0.1
        print(f"Battery Voltage: {battery_voltage:.2f} V")
        print(f"Battery SOC: {battery_soc} %")
        print(f"Battery Charging Amps: {battery_charging_amps:.2f} A")
else:
    print("Failed to connect")
