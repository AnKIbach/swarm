#!/usr/bin/env python2

"""
ROS node responsible for decoding multicast UDP messages and distributing
them internally to ROS
"""

import sys
import os

sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))

import rospy
import socket
import zlib

from communication.msg import SwarmStatus
from communication.msg import SwarmCommand
from communication.msg import SwarmOdometry

from solo_udp import MulticastListener
import json_serialization
import msg_types

class UdpListener(object):
    """
    This class should be instantiated with a list of topics that
    decoded UDP messages should be published to
    """
    def __init__(self, mcast_grp, mcast_port, timeout=1.0):
        self._listener = MulticastListener(mcast_grp, mcast_port,
                timeout=timeout)
        self._odometryPublisher     = rospy.Publisher("/gcs/comm/data/odometry",  SwarmOdometry,  queue_size = 50)
        self._systemStatusPublisher = rospy.Publisher("/gcs/comm/data/status",    SwarmStatus,    queue_size = 20)
        self._swarmCommandPublisher = rospy.Publisher("/gcs/comm/data/order_ack",  SwarmCommand,   queue_size = 20)

    def _readHeader(self, msg):
        return json_serialization.json2Header(msg["header"])

    def _publishPosition(self, json_msg):
        """
        Helper method to publish decoded JSON messages
        """
        odometryMsg = json_serialization.json2SwarmOdometry(json_msg)

        self._odometryPublisher.publish(odometryMsg)

    def _publishStatus(self, json_msg):
        """
        Helper method to publish decoded JSON messages
        """

        uavStatusMsg = json_serialization.json2SwarmStatus(json_msg)

        self._systemStatusPublisher.publish(uavStatusMsg)


    def _publishSwarmCommand(self, json_msg):
        """
        Method for publishing a SwarmCommand ROS message
        """

        msg = json_serialization.json2SwarmCommand(json_msg)

        self._swarmCommandPublisher.publish(msg)
         
    def run(self):
        while not rospy.is_shutdown():
            try:
                msg = self._listener.listen()

                header = self._readHeader(msg)
                #print (header)
                if header.msgType == msg_types.MsgType.ODOMETRY:
                     self._publishPosition(msg)
                if header.msgType == msg_types.MsgType.UAVSTATUS:
                     self._publishStatus(msg)
                if header.msgType == msg_types.MsgType.SWARM_COMMAND:
                     self._publishSwarmCommand(msg)
            except socket.timeout:
                #This is expected, we need to periodically check if ROS
                #is shutting down and we do this by way of timeout
                pass
            except zlib.error as e:
                rospy.logwarn("Zlib error while decoding {!s}".format(e))
            except socket.error, e:
                if not rospy.is_shutdown():
                    #We are not shutting down which means this is most likely
                    #not caused by ROS
                    rospy.logerr("Got general socket error: {!s}".format(e))
            except (KeyError, ValueError, TypeError), e:
                rospy.logerr("Could not convert JSON to Odometry:\
                        Error: {!s}"
                        .format(e))
            except IOError as e:
                rospy.logwarn(e)

def main():
    rospy.init_node('gcs_rx_node', anonymous=True)
    rospy.logdebug("Started udp_pos_gcs node")

    mcast_grp = rospy.get_param('~mcast_addr', "225.0.0.25")
    mcast_port = rospy.get_param('~mcast_port', 4243)

    rospy.loginfo("Using multicast group: {}:{}".format(mcast_grp, mcast_port))

    pub = UdpListener(mcast_grp, mcast_port)
    rospy.loginfo("Starting run")
    pub.run()
    rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
