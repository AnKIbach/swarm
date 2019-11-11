#!/usr/bin/env python
import time
import rospy

from swarm.msg import SwarmHeader
from swarm.msg import BoatOdometry
from swarm.msg import BoatStatus
# from swarm.msg import Position
# from swarm.msg import Movement

from Boat_ID import get_ID
# from Classes.GPS_class import GPS
# from Classes.Vector_class import Vector

class Talker:
    '''Sends data from autopilot to ROS topics'''
    def __init__(self):

        topic_status  = "autopilot/status"
        topic_current = "autopilot/current"
        topic_wanted  = "autopilot/wanted"
        topic_change  = "autopilot/change"

        self.pub_status  = rospy.Publisher(topic_status, BoatStatus, queue_size=100)
        self.pub_current = rospy.Publisher(topic_current, BoatOdometry, queue_size=10)
        self.pub_wanted  = rospy.Publisher(topic_wanted, BoatOdometry, queue_size=10)
        self.pub_change  = rospy.Publisher(topic_change, BoatOdometry, queue_size=10)

        self.header             = SwarmHeader()
        self.current_status     = BoatStatus()
        self.current_data       = BoatOdometry()
        self.wanted_data        = BoatOdometry()
        self.change_data        = BoatOdometry()

        self.BOAT_ID = get_ID()

    def __call__(self, 
                current_movement, 
                current_position, 
                wanted_movement,
                change_movement):

        self.current_data.header = self._get_header()
        self.current_data.movement.velocity  = current_movement.magnitude
        self.current_data.movement.bearing   = current_movement.angle
        self.current_data.position.latitude  = current_position.lat
        self.current_data.position.longitude = current_position.lon

        self.pub_current.publish(self.current_data)
        
        self._publish_wanted(wanted_movement)
        self._publish_change(change_movement)
