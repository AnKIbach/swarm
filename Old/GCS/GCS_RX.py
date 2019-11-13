#!/usr/bin/env python

"""
ROS node responsible for decoding multicast UDP messages and distributing
them internally
"""

import sys
import socket

import Interpreter

from Udp_GListener import GCSListener
from Multicast.Multicaster import MulticastListener


def main():

    mcast_grp  = "225.0.0.25"
    mcast_port = 4243

    print("Using multicast group: {}:{}".format(mcast_grp, mcast_port))

    pub = GCSListener(mcast_grp, mcast_port)
    while True:
        try:
            pub.run()
        except KeyboardInterrupt as e:
            print("Exiting with error: {}".format(e))
            sys.exit()

if __name__ == '__main__':
    main()
