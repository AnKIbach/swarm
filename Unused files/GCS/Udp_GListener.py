import sys
# import os

import socket
import zlib

import Interpreter

from Classes.Msg_type import MsgType
from Multicast.Multicaster import MulticastListener


class GCSListener(object): #fix publisher - no need to publish.
    """
    This class should be instantiated with a list of topics that
    decoded UDP messages should be published to
    """
    def __init__(self, mcast_grp, mcast_port, timeout=1.0):
        self._listener = MulticastListener(mcast_grp, mcast_port,
                timeout=timeout)
                
    def _read_header(self, msg):
        return Interpreter.header2GCS(msg["header"])

    def _handle_odometry(self, json_msg):
        """
        Helper method to publish decoded JSON messages
        """
        odometryMsg = Interpreter.odometry2GCS(json_msg)
        return odometryMsg
        #Some way to forward data to GCS

    def _handle_status(self, json_msg):
        """
        Helper method to publish decoded JSON messages
        """

        uavStatusMsg = Interpreter.status2GCS(json_msg)

    def _send_swarmCommand(self, noe):
        """
        Method for publishing a SwarmCommand ROS message
        """

        Interpreter.GCS2command(noe)
         
    def run(self):
        while True:
            try:
                msg = self._listener.listen()

                header = self._read_header(msg)
                #print (header)
                if header.msgType == MsgType.ODOMETRY:
                    odometry = self._handle_odometry(msg)
                    return odometry
                if header.msgType == MsgType.BOAT_STATUS:
                    self._handle_status(msg)
                if header.msgType == MsgType.SWARM_COMMAND:
                    self._send_swarmCommand(msg)
            except socket.timeout:
                #This is expected, we need to periodically check if ROS
                #is shutting down and we do this by way of timeout
                pass