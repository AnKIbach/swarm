#!/usr/bin/env python

"""
ROS node responsible for decoding multicast UDP messages and distributing
them internally to ROS
"""

import sys
import os

# sys.path.append(
#     os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))

import rospy

from Udp_Listener import Listener

def main():
    rospy.init_node('uav_rx_node', anonymous=True)
    rospy.logdebug("Started udp_pos node")

    mcast_grp = rospy.get_param('~mcast_addr', "225.0.0.25")
    mcast_port = rospy.get_param('~mcast_port', 4243)

    rospy.loginfo("Using multicast group: {}:{}".format(mcast_grp, mcast_port))

    pub = Listener(mcast_grp, mcast_port)

    rospy.loginfo("Starting run")
    pub.run()
    rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()