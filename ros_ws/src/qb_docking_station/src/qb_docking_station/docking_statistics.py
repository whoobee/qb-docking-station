#!/usr/bin/env python3
import os
import time
import socket
import collections.abc

# Docking Statistics class
class DockingStatistics(object):
    SOCKET = None
    VERBOSE = False
    URL = None
    PORT = None
    # Init statistics class
    def __init__(self, _url, _port, _verbose):
        # Init configuration
        self.SOCKET = socket.socket()
        self.VERBOSE = _verbose
        self.URL = _url
        self.PORT = _port
        # Open connection to Carbon server
        try:
            self.SOCKET.connect( (_url, _port) )
        except socket.error:
            raise SystemExit("Couldn't connect to %(server)s on port %(port)d, is carbon-cache.py running?" % { 'server':_url, 'port':_port })


    # Denit statistics class
    def deinit(self):
        self.SOCKET.close()


    # Report value
    def report(self, _name, _value, _timestamp=-1):
        lines = []
        if(_timestamp < 0):
            now = int(time.time())
        else:
            now = _timestamp
        # Create message lines
        # Check if we provided an array as value
        if(isinstance(_value, collections.abc.Sequence)):
            for index in range(len(_value)):
                lines.append(_name + " %s %d" % (_value[index], now))
        else:
            lines.append(_name + " %s %d" % (_value, now))
        # Pack the lines into a message
        message = '\n'.join(lines) + '\n' #all lines must end in a newline
        # Print for debugging
        if (self.VERBOSE): print("[INFO] Sending Message to Carbon(%s,%d): %s" % (self.URL, self.PORT, message))
        # Send value to Carbon
        self.SOCKET.sendall(message.encode())

# FOR TEST ONLY; TO BE REMOVED!!!
import random

ds = DockingStatistics("localhost", 2003, True)
try:
    while True:
        ds.report("local.random.diceroll",[random.randint(0,6)])
        time.sleep(30)
except KeyboardInterrupt:
    ds.deinit()
    print ('[INFO] Exit!')