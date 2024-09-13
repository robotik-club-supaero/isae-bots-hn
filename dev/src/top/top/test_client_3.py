#!/usr/bin/env python3

import socket
import time

from top_const import TopServerRequest, TOPSERVER_PORT

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = TOPSERVER_PORT  # The port used by the server

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    
    try :
        while True:
            s.sendall(bytes([TopServerRequest.REQUEST_READ_TRIGGER]))
            data = s.recv(1024)
            
            print(f"Received {list(bytes(data))}")
            
            time.sleep(0.1)
            
            
    except BrokenPipeError: #NOTE happens if the server shuts down by Ctrl-C
        print("Server shut down")