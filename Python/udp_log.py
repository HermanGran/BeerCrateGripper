import socket
import datetime

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('', 4444))
print('Listening on UDP :4444')

while True:
    data, addr = s.recvfrom(1024)
    ts = datetime.datetime.now().strftime('%H:%M:%S')
    print(f'{ts} [{addr[0]}] {data.decode().strip()}')