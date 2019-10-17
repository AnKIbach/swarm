#!/usr/bin/env python

"""
ROS node responsible for decoding multicast UDP messages and distributing
them internally
"""

import sys
import os

# sys.path.append(
#     os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))

import rospy
import socket

import Json
import Msg_type

from Udp_GListener import GCSListener
from Multicast.Multicaster import MulticastListener

from autopilot.msg import BoatOdometry
from autopilot.msg import BoatStatus
from autopilot.msg import SwarmCommand

def main():
    rospy.init_node('gcs_rx_node', anonymous=True)
    rospy.logdebug("Started udp_pos_gcs node")

    mcast_grp = rospy.get_param('~mcast_addr', "225.0.0.25")
    mcast_port = rospy.get_param('~mcast_port', 4243)

    rospy.loginfo("Using multicast group: {}:{}".format(mcast_grp, mcast_port))

    pub = GCSListener(mcast_grp, mcast_port)
    rospy.loginfo("Starting run")
    pub.run()
    rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
