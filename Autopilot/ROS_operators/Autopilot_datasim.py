#!/usr/bin/env python
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
        self.bearing = 160

    def __call__(self):
        self.header = self._get_header()

        
        # self._send1(0)
        # self._send2(1)
        self._send3(2)
        self._send4(3)
        
    def _send1(self, bid):
        self.curr.header = self.header
        self.curr.header.id = bid

        self.curr.movement.velocity  = self.speed
        self.curr.movement.bearing   = self.bearing
        self.curr.position.latitude  = 60.394333
        self.curr.position.longitude = 5.265925

        self.pub_current.publish(self.curr)

    def _send2(self, bid):
        self.curr.header = self.header
        self.curr.header.id = bid

        self.curr.movement.velocity  = self.speed
        self.curr.movement.bearing   = self.bearing
        self.curr.position.latitude  = 60.394333
        self.curr.position.longitude = 5.265970

        self.pub_current.publish(self.curr)

    def _send3(self, bid):
        self.curr.header = self.header
        self.curr.header.id = bid

        self.curr.movement.velocity  = self.speed
        self.curr.movement.bearing   = self.bearing
        self.curr.position.latitude  = 60.394367
        self.curr.position.longitude = 5.265970

        self.pub_current.publish(self.curr)

    def _send4(self, bid):
        self.curr.header = self.header
        self.curr.header.id = bid

        self.curr.movement.velocity  = self.speed
        self.curr.movement.bearing   = self.bearing
        self.curr.position.latitude  = 60.394367
        self.curr.position.longitude = 5.265925

        self.pub_current.publish(self.curr)

    def _get_header(self, msgtype = 1):

        time_now = rospy.get_rostime()
        self.header.secs    = time_now.secs
        self.header.nsecs   = time_now.nsecs
        self.header.msgType = msgtype

        return self.header


    #     statePX     = "/mavros/state" 
    #     GPSPX       = "/mavros/global_position/raw/fix"
    #     compassPX   = "/mavros/vfr_hud"
    #     velocityPX  = "/mavros/global_position/raw/gps_vel"


    #     self.pub_state  = rospy.Publisher(statePX, mavros_msgs.msg.State, queue_size=100)
    #     self.pub_current = rospy.Publisher(GPSPX, sensor_msgs.msg.NavSatFix, queue_size=10)
    #     self.pub_wanted  = rospy.Publisher(compassPX, mavros_msgs.msg.VFR_HUD, queue_size=10)
    #     self.pub_change  = rospy.Publisher(velocityPX, geometry_msgs.msg.TwistStamped, queue_size=10)

    #     self.state    = mavros_msgs.msg.State()
    #     self.position = sensor_msgs.msg.NavSatFix()
    #     self.bearing  = mavros_msgs.msg.VFR_HUD()
    #     self.speed    = geometry_msgs.msg.TwistStamped()


    # def __call__(self):
    #     self.state.connected = True
    #     self.state.mode = True

    #     self._send1(self.state)
    
    # def _send1(self,state):
    #     self.position.lat = 60.5
    #     self.position.lon = 5.26
    #     self.bearing.heading = 160.0
    #     self.speed.twist.linear.x = 0.5
    #     self.speed.twist.linear.y = 0.5

    # def _send2(self):
    #     self.position.lat = 60.5
    #     self.position.lon = 5.26
    #     self.bearing.heading = 160.0
    #     self.speed.twist.linear.x = 0.5
    #     self.speed.twist.linear.y = 0.5


