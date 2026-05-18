#!/usr/bin/env python3
# steer_monitor.py - monitor steering setpoint vs actual pot in real time
import socket, json, time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', 6003))

print(f"{'Time':>8}  {'Mode':>4}  {'Setpt':>6}  {'Actual':>6}  {'Error':>6}  {'PWM':>5}  {'Dir':>3}  {'Signal':>6}")
print("-" * 65)

while True:
    data, _ = sock.recvfrom(4096)
    msg = json.loads(data)
    s = msg.get('steering', {})
    r = msg.get('radio', {})
    if not s:
        continue
    t = time.strftime('%H:%M:%S')
    print(f"{t:>8}  {s.get('mode',0):>4}  {s.get('setpoint',0):>6.0f}  "
          f"{s.get('current',0):>6.0f}  {s.get('error',0):>6.0f}  "
          f"{s.get('pwm',0):>5.0f}  {s.get('direction','?'):>3}  "
          f"{r.get('signal','?'):>6}")