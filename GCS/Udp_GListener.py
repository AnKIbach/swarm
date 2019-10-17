import sys
# import os

import socket
import zlib

import rospy

import Json

from Msg_type import MsgType
from Multicast.Multicaster import MulticastListener

from autopilot.msg import BoatStatus
from autopilot.msg import BoatOdometry
from autopilot.msg import SwarmCommand

class GCSListener(object): #fix publisher - no need to publish.
    """
    This class should be instantiated with a list of topics that
    decoded UDP messages should be published to
    """
    def __init__(self, mcast_grp, mcast_port, timeout=1.0):
        self._listener = MulticastListener(mcast_grp, mcast_port,
                timeout=timeout)
                
        self._odometryPublisher     = rospy.Publisher("/gcs/data/odometry",  BoatOdometry,  queue_size = 50)
        self._systemStatusPublisher = rospy.Publisher("/gcs/data/status",    BoatStatus,    queue_size = 20)
        self._swarmCommandPublisher = rospy.Publisher("/gcs/data/order_ack",  SwarmCommand,   queue_size = 20)

    def _readHeader(self, msg):
        return Json.json2Header(msg["header"])

    def _publishPosition(self, json_msg):
        """
        Helper method to publish decoded JSON messages
        """
        odometryMsg = Json.json2BoatOdometry(json_msg)

        self._odometryPublisher.publish(odometryMsg)

    def _publishStatus(self, json_msg):
        """
        Helper method to publish decoded JSON messages
        """

        uavStatusMsg = Json.json2BoatStatus(json_msg)

        self._systemStatusPublisher.publish(uavStatusMsg)


    def _publishSwarmCommand(self, json_msg):
        """
        Method for publishing a SwarmCommand ROS message
        """

        msg = Json.json2SwarmCommand(json_msg)

        self._swarmCommandPublisher.publish(msg)
         
    def run(self):
        while not rospy.is_shutdown():
            try:
                msg = self._listener.listen()

                header = self._readHeader(msg)
                #print (header)
                if header.msgType == MsgType.ODOMETRY:
                     self._publishPosition(msg)
                if header.msgType == MsgType.BOAT_STATUS:
                     self._publishStatus(msg)
                if header.msgType == MsgType.SWARM_COMMAND:
                     self._publishSwarmCommand(msg)
            except socket.timeout:
                #This is expected, we need to periodically check if ROS
                #is shutting down and we do this by way of timeout
                pass
            except zlib.error as e:
                rospy.logwarn("Zlib error while decoding {!s}".format(e))
            except IOError as e:
                rospy.logwarn(e)