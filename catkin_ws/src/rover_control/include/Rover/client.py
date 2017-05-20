#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncore
import logging
import socket
import time
from settings import *


class Client(asyncore.dispatcher):
    def __init__(self, host, port):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect((host, port))
        self.msg = "B100000E/B100233E/23,12"
        self.is_writable = True

    def handle_connect(self):
        print "connected"

    def handle_close(self):
        print "closed"
        self.close()

    def handle_read(self):
        self.data = self.recv(1024)
        print(self.data[:-1])

    def handle_error(self):
        print("Server is closed...")

    def writable(self):
        self.is_writable = not self.is_writable
        return self.is_writable

    def handle_write(self):
        sent = self.send(self.msg)


if __name__ == "__main__":
    client = Client(HOST, PORT)
    asyncore.loop(timeout=0.5)

