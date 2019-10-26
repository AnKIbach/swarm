#!/usr/bin/env python
import time
import rospy
import math as m

from std_msgs.msg import Header

from autopilot.msg import SwarmHeader
from autopilot.msg import BoatOdometry
from autopilot.msg import Movement

class Talker:
    def __init__(self):
        topic_wanted  = "/swarm/behaviour/wanted"

        self.pub_wanted  = rospy.Publisher(topic_wanted, BoatOdometry, queue_size=10)

        self.wanted_data = BoatOdometry()


    def __call__(self, 
                wanted_velocity,
                wanted_bearing):
        
        # if there is need to use headers
        # self.time_now = rospy.get_rostime()
        # self.current_data.header.secs    = self.time_now.secs
        # self.current_data.header.nsecs   = self.time_now.nsecs
        # self.current_data.header.id      = BOAT_ID
        # self.current_data.header.msgType = 1

        # self.wanted_data.header = self.current_data.header
        
        self.wanted_data.movement.velocity  = wanted_velocity
        self.wanted_data.movement.bearing   = wanted_bearing

        self.pub_wanted.publish(self.wanted_data)
