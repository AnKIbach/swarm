#!/usr/bin/env python
import rospy
import math as m

import std_msgs.msg 
import mavros_msgs.msg 
import sensor_msgs.msg 
import geometry_msgs.msg 

from GPS_class import GPS
from Vector_class import Vector

class talker:
    def __init__(self):
        self.pub_actual = rospy.Publisher('autopilot_actual', std_msgs.msg._Float64, queue_size=1)
        self.pub_wanted = rospy.Publisher('autopilot_wanted', std_msgs.msg._Float64, queue_size=1)
        self.pub_sent = rospy.Publisher('autopilot_sent', std_msgs.msg._Float64, queue_size=1)
        
        rospy.init_node('autopilot_publisher', anonymous=True)

    def __call__(self, wanted, actual, sent):

        self.pub_wanted.publish(wanted.magnitude)
        self.pub_actual.publish(actual.magnitude)
        self.pub_sent.publish(sent.magnitude)
