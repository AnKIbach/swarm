#!/usr/bin/env python

"""
ROS node responsible for decoding position messages and distributing them
on multicast UDP
"""

import sys
import os

# uncertain of use in code
# sys.path.append(
#     os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))

import rospy

from Classes.Udp_Publisher import PositionPublisher

from swarm.msg import BoatOdometry
from swarm.msg import BoatStatus
from swarm.msg import SwarmCommand

HEADER_FMT = 'b'

def main():
    rospy.init_node('uav_tx_node', anonymous=True)
    rospy.logdebug("Started pos_udp node")

    # defining variables for udp publisher
    mcast_grp  = rospy.get_param('~mcast_addr', "225.0.0.25")
    mcast_port = rospy.get_param('~mcast_port', 4243)
    compress   = rospy.get_param('~compression', False)
    nav_hz     = rospy.get_param('~nav_hz', 10.0)
    state_hz   = rospy.get_param('~state_hz', 1.0)
    cpu_hz     = rospy.get_param('~cpu_hz', 1.0)

    ttl = rospy.get_param('~ttl', 1) #ttl - time to live for socket

    #Definition of topic adresses for data to send
    odometry_topic    = rospy.get_param('~odometry_subscriber',      "/autopilot/current")
    status_topic      = rospy.get_param('~status_subscriber',    "/autopilot/status")

    #Initialization of publisher socket
    publisher = PositionPublisher(mcast_grp, mcast_port,
            ttl=ttl, compress=compress, nav_hz = nav_hz, state_hz=state_hz,
            cpu_hz=cpu_hz)

    #initialization of subscribers 
    rospy.Subscriber(odometry_topic,  BoatOdometry, publisher.handle_odometry)
    rospy.Subscriber(status_topic,    BoatStatus,   publisher.handle_boat_status)

    rospy.loginfo("Using multicast group: {}:{}".format(mcast_grp, mcast_port))
    rospy.loginfo("Odometry subscription: {!s}".format(odometry_topic))

    rospy.loginfo("Should output be compressed: {!s}".format(compress))
    rospy.logdebug("Time to live for UDP: {!s}".format(ttl))

    #Give control over to ROS so that Python doesn't exit
    rospy.loginfo("Starting to publish")
    rospy.spin()
    publisher.shutdown()
    rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
