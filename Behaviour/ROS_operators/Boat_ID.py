#!/usr/bin/env python

import socket

def get_ID():
    '''Finds current IP from socket host name and translates to ID'''
    IDs = {"192.168.136.61" : 0,
            "192.168.136.62" : 1,
            "192.168.136.63" : 2,
            "192.168.136.64" : 3,
            "192.168.136.65" : 4}

    # hostname = socket.gethostname()
    # IP       = socket.gethostbyname(hostname)
    IP = _get_ip()
    print(IP)
    try:
        return IDs[IP]
    
    except KeyError:
        return 0

def _get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("192.168.136.60", 80))
    return s.getsockname()[0]