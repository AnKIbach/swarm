#!/usr/bin/env python

#maybe python2 if not working

"""
ROS node responsible for decoding position messages and distributing them
on multicast UDP
"""

import sys
import os

# sys.path.append(
#     os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))

import rospy

import Json

from Multicast.Multicaster import MulticastSender

HEADER_FMT = 'b'

class PositionPublisher(object):
    """
    This class exposes two methods, "handle_nav_data" and "handle_pose_data"
    which can be used as callback functions. These functions serializes the
    given ROS messages into JSON messages before sending the data on
    UDP multicast
    """
    def __init__(self, mcast_grp, mcast_port, ttl=2, compress=False,
                 nav_hz = 10.0, state_hz=1.0, cpu_hz=1.0):
        self._msender = MulticastSender(mcast_grp, mcast_port, compress=compress)

    def shutdown(self):
        """
        Helper method to shutdown all running code
        """
        try:
            if self._publish is not None:
                self._publish.cancel()
        except:
            pass

    def handle_odometry(self, odometry):
        """
        Handle NavSatFix data to publish
        """
        self._msender.send_message( self._createOdometry(odometry) )

    def handle_boat_status(self, boatStatus):
        """
        Handle HAL callback
        """
        self._msender.send_message( self._createBoatState(boatStatus) )


    def handle_swarm_command(self, swarmCommand):
        """
        Handles SwarmCommand callback
        """

        self._msender.send_message( self._createSwarmCommand(swarmCommand) )

    def _createOdometry(self, odometry):
        msg = Json.boatOdometry2Json(odometry)
        return msg

    def _createSwarmCommand(self, swarmCommand):
        msg = Json.swarmCommand2Json(swarmCommand)
        return msg
  
    def _createBoatState(self, boatStatus):
        msg = Json.boatStatus2Json(boatStatus)
        return msg