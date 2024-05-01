#!/usr/bin/env python3

import socket
from top_const import TOPSERVER_PORT

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = TOPSERVER_PORT  # The port used by the server

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.sendall(b"Hello, world from second")
    data = s.recv(1024)

print(f"Received {data!r}")