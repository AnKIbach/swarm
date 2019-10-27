#!/usr/bin/env python

import socket
from enum import Enum

def get_ID():
    IDs = {"192.168.136.61" : 1,
            "192.168.136.62" : 2,
            "192.168.136.63" : 3,
            "192.168.136.64" : 4,
            "192.168.136.65" : 5}

    hostname = socket.gethostname()
    IP       = socket.gethostbyname(hostname)
    
    try:
        return IDs[IP]
    
    except KeyError:
        return 5


