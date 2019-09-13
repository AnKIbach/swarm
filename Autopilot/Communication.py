#!/usr/bin/env python
from Multicast.Receive import receive
from Multicast.Send import send

class communication:
    def __init__(self, my_lat = 0.0, my_lon = 0.0):
        self.lat = my_lat
        self.lon = my_lon
        self.list = [[0.0,0.0,0.0]]
        self.example_list = [[192.1,60.5,5.2],
                            [192.2, 60.4,5.2],
                            [192.3, 60.5,5.25]]
    
    def build_list(self):
        self.sender = send()
        self.receiver = receive()
            
