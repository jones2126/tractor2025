import serial

s = serial.Serial('/dev/teensy', 460800, timeout=2)
line = s.readline()
print("Received:", line)
s.close()
