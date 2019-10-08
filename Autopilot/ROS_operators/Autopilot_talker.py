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

class Talker:
    def __init__(self):
        
        autopilotStatus = "autopilot/status"
        autopilotWanted = "autopilot/wanted"
        autopilotChange = "autopilot/change"

        self.pub_status = rospy.Publisher(autopilotStatus, BoatData, queue_size=10)
        self.pub_wanted = rospy.Publisher(autopilotWanted, RuntimeData, queue_size=10)
        self.pub_change = rospy.Publisher(autopilotChange, RuntimeData, queue_size=10)

        self.autoData = BoatData()
        self.autoData.header = Header()
        self.autoData.data = RuntimeData()
        self.wanted_data = RuntimeData()
        self.change_data = RuntimeData()

    def __call__(self, 
                movement_data, 
                position_data, 
                wanted_movement,
                wanted_position,
                change_movement):

        self.autoData.header.stamp    = rospy.Time.now()
        self.autoData.header.frame_id = ''

        self.autoData.data.movement.velocity  = movement_data.magnitude
        self.autoData.data.movement.bearing   = movement_data.angle
        self.autoData.data.position.latitude  = position_data.lat
        self.autoData.data.position.longitude = position_data.lon

        self.pub_status.publish(self.autoData)
        
        self._pub_wanted(wanted_movement, wanted_position)
        self._pub_change(change_movement)

    def _pub_wanted(self, movement_data, position_data):
        self.wanted_data.movement.velocity  = movement_data.magnitude
        self.wanted_data.movement.bearing   = movement_data.angle
        self.wanted_data.position.latitude  = position_data.lat
        self.wanted_data.position.longitude = position_data.lon

        self.pub_wanted.publish(self.wanted_data)

    def _pub_change(self, output_data):
        self.change_data.movement.velocity  = output_data.magnitude
        self.change_data.movement.bearing   = output_data.angle

        self.pub_change.publish(self.change_data)



