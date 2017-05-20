
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import asyncore
import sys
import threading
import serial
from glob import glob
from settings import *


utility_names = {"SC": "Sensor Controller",
                 "BM": "Battery Management",
                 "RA": "Robotic Arm",
                 "M":  "Motor Controller",
                 "L": "LORA"}


class SerialNode(object):
    def __init__(self, server):
        self.ports = glob('/dev/ttyACM*') + glob('/dev/ttyUSB*')
        self.server = server
        self.serials = []
        self.utilities = {}
        self.msg = None
        self.msg_interval = WRITE_INTERVAL
        self.is_connected = False
        self.add_serials()
        self.add_utilities()

    def read_data(self, utility_name):
        utility = self.utilities[utility_name]

        while True:
            if utility.inWaiting:
                try:
                    data_array = utility.readline().split()
                    try:
                        serial_data = " ".join(data_array) + "\n"
                        if "L" in self.utilities:
                            try:    
                                self.utilities["L"].write(serial_data)
                            except Exception as e:
                                print(e)
                        if self.server.handler.is_connected:
                            self.server.handler.serial_data = serial_data
                    except Exception:
                        pass
                except Exception as e:
                    print(e)

    def write_data(self, utility_name):
        utility = self.utilities[utility_name]
        if utility_name == "SC": print "hey"

        while True:
            try:
                self.msg = self.server.handler.data
                self.server.handler.data = None
            except AttributeError:
                self.msg = None
            except Exception as e:
                self.msg = None

            if "L" in self.utilities and self.msg is None:
                 try:
                     self.msg = self.utilities["L"].readline()
                 except AttributeError:
                     self.msg = None
                 except Exception as e:
                     self.msg = None

            if self.msg:
                try:
                    self.msg = self.msg.split("\r\n")[0]
                    self.msg = self.msg.split("/")
                except:
                    continue

                if utility_name == "M":
                    utility.write(self.msg[0] + "\n")
                elif utility_name == "RA":
                    utility.write(self.msg[1] + "\n")
                elif utility_name == "SC":
                    try:
                        self.gps = self.msg[2] + '\n'
                        if not self.msg[2] in ('go', 'delete', 'stop'):
                            coord = [float(x) for x in self.msg[2].split(',')]
                    except:
                        coord = [0, 0]
                    if self.msg[2] in ('go', 'delete', 'stop') or (coord[0] != 0 and coord[1] != 0):
                        utility.write(self.gps)

                utility.flush()
                self.msg = None
                time.sleep(self.msg_interval)


    def add_serials(self):
        for port in self.ports:
            try:
                self.serials.append(serial.Serial(port, 115200))
            except Exception as e:
                print(e)

        if not self.serials:
            print("no serials")
        
        print(self.serials)

    def add_utilities(self):
        serial_no = 0
        num_serials = len(self.serials)

        while serial_no != num_serials:
            utility = self.serials[serial_no]
            utility.write("AFAF0000AF8003020000920D0A".decode("hex"))
            serial_data = utility.readline().split("\r\n")
            serial_data = serial_data[0].split(',')
            utility_name = str(serial_data[0])

            if utility_name in utility_names:
                print(utility_name + " Identified")

                self.utilities[utility_name] = utility

                serial_no += 1

    def run_server(self):
        try:
            server_thread = threading.Thread(
                    target=asyncore.loop,
                    args=())
            server_thread.daemon = True
            server_thread.start()
        except Exception as e:
            print(e)
        
        self.is_connected = True

    def run(self):
        print("running")
        try:
            for utility_name in self.utilities:
                if not utility_name == "SC":
                    continue
                try:
                    reading_thread = threading.Thread(
                        target=self.read_data, args=(utility_name, ))
                    reading_thread.daemon = True
                    reading_thread.start()
                except Exception as e:
                    print(e)

            for utility_name in self.utilities:
                if not utility_name in ("M", "RA", "SC"):
                    continue
                try:
                    writing_thread = threading.Thread(
                        target=self.write_data,
                        args=(utility_name, ))
                    writing_thread.daemon = True
                    writing_thread.start()
                except Exception as e:
                    print(e)
            
            if not self.is_connected:
                self.run_server()

            while True:
                self.current_ports = glob('/dev/ttyACM*') + glob('/dev/ttyUSB*')
                if self.current_ports != self.ports:
                    self.ports = self.current_ports
                    self.serials = []
                    self.utilities = {}
                    self.add_serials()
                    self.add_utilities()
                    self.run()
                    time.sleep(2)
                    continue


                for utility_name in self.utilities:
                    if not self.utilities[utility_name].isOpen():
                        self.serials = []
                        self.utilities = {}
                        self.add_serials()
                        self.add_utilities()
                        self.run()
                        break

                time.sleep(2)
        except Exception:
            print("Exiting...")
            sys.exit()
