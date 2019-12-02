#!/usr/bin/env python
'''
This module contains functions for opening 
and maintaining a udp publisher socket

Questions: anhellesnes@fhs.mil.no
'''

import sys
import os

import rospy

import Json

from Multicaster import MulticastSender

HEADER_FMT = 'b'

class PositionPublisher(object):
    '''
    This class exposes two methods, "handle_nav_data" and "handle_pose_data"
    which can be used as callback functions. These functions serializes the
    given ROS messages into JSON messages before sending the data on
    UDP multicast
    '''
    def __init__(self, mcast_grp, mcast_port, ttl=2, compress=False,
                 nav_hz = 10.0, state_hz=1.0, cpu_hz=1.0):
        self._msender = MulticastSender(mcast_grp, mcast_port, compress=compress)

    def shutdown(self):
        '''
        Helper method to shutdown all running code
        '''
        try:
            if self._publish is not None:
                self._publish.cancel()
        except:
            pass

    def handle_odometry(self, odometry):
        '''Handles Boat odometry data to publish

        Args:
            ROS BoatOdometry msg object
        '''
        self._msender.send_message( self._createOdometry(odometry) )

    def handle_boat_status(self, boatStatus):
        '''Handles Boat status data to publish

        Args:
            ROS BoatStatus msg object
        '''
        self._msender.send_message( self._createBoatState(boatStatus) )


    def handle_swarm_command(self, swarmCommand):
        '''Handles swarm command data to publish

        Args:
            ROS SwarmCommand msg object
        '''

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