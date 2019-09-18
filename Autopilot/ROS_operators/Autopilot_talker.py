#!/usr/bin/env python
import time
import rospy
import math as m
from datetime import datetime

from std_msgs.msg import Header

from GPS_class import GPS
from Vector_class import Vector

from autopilot.msg import RuntimeData
from autopilot.msg import BoatData

class talker:
    def __init__(self):

        self.pub_status = rospy.Publisher('autopilot/actual', BoatData, queue_size=10)
        self.pub_wanted = rospy.Publisher('autopilot/wanted', RuntimeData, queue_size=10)
        self.pub_change = rospy.Publisher('autopilot/change', RuntimeData, queue_size=10)

        self.autoData = BoatData()
        self.autoData.header = Header()
        self.autoData.data = RuntimeData()

    def __call__(self, movement_data, position_data, nav_status = False, ardu_status = False):
        self.autoData.header.stamp    = rospy.Time.now()
        self.autoData.header.frame_id = ''

        self.autoData.data.velocity  = movement_data.magnitude
        self.autoData.data.bearing   = movement_data.angle
        self.autoData.data.latitude  = position_data.lat
        self.autoData.data.longitude = position_data.lon

        self.autoData.NavReady     = nav_status
        self.autoData.ArduinoReady = ardu_status

        self.pub_status.publish(self.autoData)

    def _pub_wanted(self, movement_data, position_data):
        pass

    def _pub_change(self, movement_data, position_data):
        pass



