#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncore
import logging
from server import EchoServer
from serial_com import SerialNode
from settings import *


def main():
    tcp_server = EchoServer(HOST, PORT)
    serial_node = SerialNode(tcp_server)
    serial_node.run()


if __name__ == "__main__":
    main()
