#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncore
import logging
import socket
import time
from settings import *


class EchoHandler(asyncore.dispatcher_with_send):
    serial_data = None

    def writable(self):
        return True

    def handle_write(self):
        if self.serial_data:
            sent = self.send(self.serial_data)
            logging.info('Send "%s" to client.' % self.serial_data.split()[0])
            time.sleep(0.1)

    def handle_read(self):
        self.data = self.recv(1024)
        logging.info('Read "%s" from client' % self.data)


class EchoServer(asyncore.dispatcher):
    def __init__(self, host, port):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        logging.info('Create a server')
        self.data = "123"
        self.set_reuse_addr()
        self.bind((host, port))
        self.listen(5)
        logging.info('Listen')
        print("listening")

    def handle_accept(self):
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            logging.info('A client connected: %s' % repr(addr))
            print('Incoming connection from %s' % repr(addr))
            self.handler = EchoHandler(sock)
            logging.warning('A client has disconnected: %s' % repr(addr))


if __name__ == "__main__":
    server = EchoServer(HOST, PORT)
    asyncore.loop()
