#!/usr/bin/env python
import time
import rospy
import math as m

from std_msgs.msg import Header

from GPS_class import GPS
from Vector_class import Vector

from autopilot.msg import SwarmHeader
from autopilot.msg import BoatOdometry
from autopilot.msg import Position
from autopilot.msg import Movement

#find way to get dynamically from base
BOAT_ID = 1

class Talker:
    def __init__(self):

        autopilotStatus = "autopilot/status"
        autopilotWanted = "autopilot/wanted"
        autopilotChange = "autopilot/change"

        self.pub_status = rospy.Publisher(autopilotStatus, BoatOdometry, queue_size=10)
        self.pub_wanted = rospy.Publisher(autopilotWanted, BoatOdometry, queue_size=10)
        self.pub_change = rospy.Publisher(autopilotChange, BoatOdometry, queue_size=10)

        self.autoData           = BoatOdometry()
        self.autoData.header    = SwarmHeader()
        self.autoData.position  = Position()
        self.autoData.movement  = Movement()
        self.wanted_data        = BoatOdometry()
        self.change_data        = BoatOdometry()

    def __call__(self, 
                current_movement, 
                current_position, 
                wanted_movement,
                wanted_position,
                change_movement):

        self.time_now = rospy.get_rostime()
        self.autoData.header.secs    = self.time_now.secs
        self.autoData.header.nsecs   = self.time_now.nsecs
        self.autoData.header.id      = BOAT_ID

        self.autoData.movement.velocity  = current_movement.magnitude
        self.autoData.movement.bearing   = current_movement.angle
        self.autoData.position.latitude  = current_position.lat
        self.autoData.position.longitude = current_position.lon

        self.pub_status.publish(self.autoData)
        
        self._pub_wanted(wanted_movement, wanted_position)
        self._pub_change(change_movement)

    def _pub_wanted(self, movement_data, position_data):

        self.wanted_data.header.secs    = self.time_now.secs
        self.wanted_data.header.nsecs   = self.time_now.nsecs
        self.wanted_data.header.id      = BOAT_ID
        
        self.wanted_data.movement.velocity  = movement_data.magnitude
        self.wanted_data.movement.bearing   = movement_data.angle
        self.wanted_data.position.latitude  = position_data.lat
        self.wanted_data.position.longitude = position_data.lon

        self.pub_wanted.publish(self.wanted_data)

    def _pub_change(self, output_data):

        self.change_data.header.secs    = self.time_now.secs
        self.change_data.header.nsecs   = self.time_now.nsecs
        self.change_data.header.id      = BOAT_ID

        self.change_data.movement.velocity  = output_data.magnitude
        self.change_data.movement.bearing   = output_data.angle

        self.pub_change.publish(self.change_data)



