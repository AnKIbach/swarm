#!/usr/bin/env python
'''
This class with a ROS publisher is used to simulate nav data

Questions: anhellesnes@fhs.mil.no
'''

import time
import rospy

import std_msgs.msg 
import mavros_msgs.msg 
import sensor_msgs.msg 
import geometry_msgs.msg

from swarm.msg import SwarmHeader
from swarm.msg import BoatOdometry
from swarm.msg import BoatStatus

class Sim:
    
    def __init__(self):
        topic_current = "autopilot/current"

        self.pub_current = rospy.Publisher(topic_current, BoatOdometry, queue_size=10)

        self.header = SwarmHeader()
        self.status = BoatStatus()
        self.curr = BoatOdometry()

        self.speed = 0.5
        self.bearing = 180

    def __call__(self):
        self.header = self._get_header()

        
        self._send1(0)
        self._send2(1)
        self._send3(2)
        self._send4(3)
        
    def _send1(self, bid):
        self.curr.header = self.header
        self.curr.header.id = bid

        self.curr.movement.velocity  = self.speed
        self.curr.movement.bearing   = 340.0
        self.curr.position.latitude  = 60.394333
        self.curr.position.longitude = 5.265925

        self.pub_current.publish(self.curr)

    def _send2(self, bid):
        self.curr.header = self.header
        self.curr.header.id = bid

        self.curr.movement.velocity  = self.speed
        self.curr.movement.bearing   = 340.0
        self.curr.position.latitude  = 60.394240
        self.curr.position.longitude = 5.265839

        self.pub_current.publish(self.curr)

    def _send3(self, bid):
        self.curr.header = self.header
        self.curr.header.id = bid

        self.curr.movement.velocity  = self.speed
        self.curr.movement.bearing   = 340.0
        self.curr.position.latitude  = 60.394240
        self.curr.position.longitude = 5.265930

        self.pub_current.publish(self.curr)

    def _send4(self, bid):
        self.curr.header = self.header
        self.curr.header.id = bid

        self.curr.movement.velocity  = self.speed
        self.curr.movement.bearing   = 340.0
        self.curr.position.latitude  = 60.394279
        self.curr.position.longitude = 5.265884

        self.pub_current.publish(self.curr)

    def _get_header(self, msgtype = 1):

        time_now = rospy.get_rostime()
        self.header.secs    = time_now.secs
        self.header.nsecs   = time_now.nsecs
        self.header.msgType = msgtype

        return self.header
