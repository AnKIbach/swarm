#!/usr/bin/env python
'''
This class is used to set boat_id based on IP-adress

Questions: anhellesnes@fhs.mil.no
'''


import socket

def get_ID():
    '''Helper function to return ID to boat based on current IP'''

    IDs = {"192.168.136.61" : 0,
            "192.168.136.62" : 1,
            "192.168.136.63" : 2,
            "192.168.136.64" : 3,
            "192.168.136.65" : 4}

    IP = _get_ip()
    try:
        return IDs[IP]
    
    except KeyError:
        return 0

def _get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("192.168.136.60", 80))
    return s.getsockname()[0]