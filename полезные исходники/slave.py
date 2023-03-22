#!/usr/bin/env python3
import socket
class test:
    def __init__(self):
        self.socket = socket.socket()
        self.socket.connect(('192.168.31.128', 4000))
        while True:
            data = self.socket.recv(1024)
            print(data.decode())


if __name__ == "__main__":
    controller = test()
