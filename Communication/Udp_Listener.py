import sys
# import os

import socket
import zlib

import rospy

import Msg_type
import Json

from Multicast.Multicaster import MulticastListener

class Listener(object):
    """
    This class should be instantiated with a list of topics that
    decoded UDP messages should be published to
    """
    def __init__(self, mcast_grp, mcast_port, timeout=1.0):
        self._listener = MulticastListener(mcast_grp, mcast_port,
                timeout=timeout)
        self._odometryPublisher     = rospy.Publisher("/swarm/comm/data/odometry",      SwarmOdometry,  queue_size = 50)
        self._systemStatusPublisher = rospy.Publisher("/swarm/comm/data/status",        SwarmStatus,    queue_size = 20)
        self._swarmCommandPublisher = rospy.Publisher("/swarm/comm/data/order_ack",   SwarmCommand,  queue_size = 20)

    def _readHeader(self, msg):
        return Json.json2Header(msg["header"])

    def _publishOdometry(self, json_msg):
        """
        Helper method to publish decoded JSON messages
        """
        odometryMsg = Json.json2SwarmOdometry(json_msg)
        self._odometryPublisher.publish(odometryMsg)
    
    def _publishStatus(self, json_msg):
        """
        Helper method to publish decoded JSON messages
        """

        uavStatusMsg = Json.json2SwarmStatus(json_msg)
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

                if header.msgType == msg_types.MsgType.ODOMETRY:
                    self._publishOdometry(msg)
                if header.msgType == msg_types.MsgType.UAVSTATUS:
                    self._publishStatus(msg)
                #if header.msgType == msg_types.MsgType.SWARM_COMMAND:
                #    self._publishSwarmCommand(msg)
            except socket.timeout:
                #This is expected, we need to periodically check if ROS
                #is shutting down and we do this by way of timeout
                pass
            except zlib.error as e:
                rospy.logwarn("Zlib error while decoding {!s}".format(e))
            # except socket.error, e:
            #     if not rospy.is_shutdown():
            #         #We are not shutting down which means this is most likely
            #         #not caused by ROS
            #         rospy.logerr("Got general socket error: {!s}".format(e))
            # except (KeyError, ValueError, TypeError), e:
            #     rospy.logerr("Could not convert JSON to Odometry:\
            #             Error: {!s}"
            #             .format(e))
            except IOError as e:
                rospy.logwarn(e)
