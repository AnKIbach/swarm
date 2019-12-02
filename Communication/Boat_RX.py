#!/usr/bin/env python
'''
Written by Andreas Hellesnes, Norwegian Naval Academy 2019

Boat_RX.py
ROS node responsible for decoding multicast UDP messages and distributing
them internally to ROS

Questions: anhellesnes@fhs.mil.no
'''

import sys
import os

import rospy

from Classes.Udp_Listener import Listener

def main():
    #initialize ROS node
    rospy.init_node('uav_rx_node', anonymous=True)
    rospy.logdebug("Started udp_pos node")

    mcast_grp  = rospy.get_param('~mcast_addr', "225.0.0.25")
    mcast_port = rospy.get_param('~mcast_port', 4243)

    rospy.loginfo("Using multicast group: {}:{}".format(mcast_grp, mcast_port))
    #initiate listener socket
    pub = Listener(mcast_grp, mcast_port)

    #Start listening and handling data from multicast
    rospy.loginfo("Starting run")
    pub.run()
    rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
