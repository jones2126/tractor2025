import socket, json, time
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 6003))
while True:
    data, _ = sock.recvfrom(4096)
    print(json.loads(data))
    time.sleep(0.01)