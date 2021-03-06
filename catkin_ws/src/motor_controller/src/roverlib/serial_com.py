#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import asyncore
import logging
import sys
import threading
import serial
from glob import glob
from settings import *


utility_names = {"SC": "Sensor Controller",
                 "BM": "Battery Management",
                 "RA": "Robotic Arm",
                 "MC": "Motor Controller",
                 "LR": "LORA"}


class SerialNode(object):
    def __init__(self, server):
        self.ports = glob('/dev/ttyACM*') + glob('/dev/ttyUSB*')
        self.server = server
        self.serials = []
        self.utilities = {}
        self.msg = None
        self.msg_interval = WRITE_INTERVAL
        logging.info('Create a serial node')
        self.add_serials()
        self.add_utilities()

    def read_data(self, utility_name):
        print("*** reading...")
        while True:
            utility = self.utilities[utility_name]
            if utility.inWaiting:
                utility.flush()
                try:
                    logging.info('Waiting data from "%s"' % utility_name)
                    data_array = utility.readline().split()
                    try:
                        serial_data = " ".join(data_array) + "\n"
                        self.server.handler.serial_data = serial_data
                        logging.info('Send "%s" from "%s"',
                                     serial_data.split('\n')[0], utility_name)
                    except Exception:
                        logging.warning('No connection detected!')
                        print("No connection detected!")
                        time.sleep(5)
                except Exception as e:
                    print(e)
                    logging.info('"%s" has faced a problem: %s',
                                 utility_name, e)

    def write_data(self, utility_name):
        print("*** writing...")
        while True:
            utility = self.utilities[utility_name]
            try:
                self.msg = self.server.handler.data
            except AttributeError:
                self.msg = None
            except Exception as e:
                print(e)

            if self.msg:
                print("writing: %s" % self.msg)
                utility.write(self.msg)
                logging.info('Write "%s" to "%s"' % (self.msg, utility_name))
                self.msg = None
                time.sleep(self.msg_interval)

    def add_serials(self):
        print("*** Adding serials...")
        logging.info('Adding serials')
        for port in self.ports:
            try:
                self.serials.append(serial.Serial(port, 115200))
                logging.info('Add serial from port: %s', port)
            except Exception as e:
                print(e)
                logging.error(e)

        if not self.serials:
            logging.warning('Could not find a serial!')

    def add_utilities(self):
        print("*** Initializing...")
        logging.info('Initializing')
        time.sleep(1)
        serial_no = 0
        num_serials = len(self.serials)
        logging.info('Search for %d serial' % num_serials)
        logging.info('Adding utilities')

        while serial_no != num_serials:
            logging.info('Waiting a response from no: %d', serial_no)
            serial_data = self.serials[serial_no].readline().split("\r\n")
            logging.info('Read the serial data %s' % serial_data)
            serial_data = serial_data[0].split(',')
            utility_name = str(serial_data[0])

            if utility_name in utility_names:
                logging.info('%s is connected.' % utility_name)
                print(utility_name + " Identified")
                print("Connected to " +
                      utility_names[utility_name] +
                      ", initializing")

                utility = self.serials[serial_no]
                time.sleep(.5)

                utility.flushInput()
                utility.flushOutput()
                self.utilities[utility_name] = utility
                logging.info('Add %s to utilities' % utility_name)

                serial_no += 1
            else:
                logging.warning(
                    '%s is not a valid utility name' % utility_name)

    def run_server(self):
        try:
            server_thread = threading.Thread(
                    target=asyncore.loop,
                    args=())
            server_thread.daemon = True
            server_thread.start()
            logging.info('Start Server Thread')
        except Exception as e:
            print(e)
            logging.error('Server thread had an error: "%s"' % e)

    def run(self):
        try:
            for utility_name in self.utilities:
                try:
                    reading_thread = threading.Thread(
                        target=self.read_data, args=(utility_name, ))
                    reading_thread.daemon = True
                    reading_thread.start()
                    logging.info('Start %s Reading Thread.' % utility_name)
                except Exception as e:
                    print(e)
                    logging.error('Reading thread had an error: "%s"' % e)

            for utility_name in self.utilities:
                try:
                    writing_thread = threading.Thread(
                        target=self.write_data,
                        args=(utility_name, ))
                    writing_thread.daemon = True
                    writing_thread.start()
                    logging.info('Start %s Writing Thread.' % utility_name)
                except Exception as e:
                    print(e)
                    logging.error('Writing thread had an error: "%s"' % e)

            self.run_server()            

            while True:
                time.sleep(1)
        except Exception:
            logging.info('Exiting...')
            print("Exiting...")
            sys.exit()
