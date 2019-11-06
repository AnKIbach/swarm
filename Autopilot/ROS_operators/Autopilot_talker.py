#!/usr/bin/env python
import time
import rospy

# from swarm.msg import SwarmHeader
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

        self.pub_status  = rospy.Publisher(topic_status, BoatStatus, queue_size=10)
        self.pub_current = rospy.Publisher(topic_current, BoatOdometry, queue_size=10)
        self.pub_wanted  = rospy.Publisher(topic_wanted, BoatOdometry, queue_size=10)
        self.pub_change  = rospy.Publisher(topic_change, BoatOdometry, queue_size=10)

        self.current_status     = BoatStatus()
        self.current_data       = BoatOdometry()
        # self.current_data.header    = SwarmHeader()
        # self.current_data.position  = Position()
        # self.current_data.movement  = Movement()
        self.wanted_data        = BoatOdometry()
        self.change_data        = BoatOdometry()

        self.BOAT_ID = get_ID()

    def __call__(self, 
                current_movement, 
                current_position, 
                wanted_movement,
                change_movement):

        self.time_now = rospy.get_rostime()
        self.current_data.header.secs    = self.time_now.secs
        self.current_data.header.nsecs   = self.time_now.nsecs
        self.current_data.header.id      = self.BOAT_ID
        self.current_data.header.msgType = 1

        self.current_data.movement.velocity  = current_movement.magnitude
        self.current_data.movement.bearing   = current_movement.angle
        self.current_data.position.latitude  = current_position.lat
        self.current_data.position.longitude = current_position.lon

        self.pub_current.publish(self.current_data)
        
        self._publish_wanted(wanted_movement)
        self._publish_change(change_movement)

    def _publish_wanted(self, movement_data):

        self.wanted_data.header = self.current_data.header
        
        self.wanted_data.movement.velocity = movement_data.magnitude
        self.wanted_data.movement.bearing  = movement_data.angle

        self.pub_wanted.publish(self.wanted_data)

    def _publish_change(self, output_data):

        self.change_data.header = self.current_data.header

        self.change_data.movement.velocity  = output_data.magnitude
        self.change_data.movement.bearing   = output_data.angle

        self.pub_change.publish(self.change_data)

    def publish_status(self, status):
        '''Publishes status of boat to ROS topic given'''
        self.current_status.header = self.current_data.header

        self.current_status.fcuMode            = 0
        self.current_status.fcuStatus          = True
        self.current_status.timeSinceLaunch    = 0.0
        self.current_status.distanceFromLaunch = 0.0
        self.current_status.numGpsSatelites    = 10

        self.current_status.pixhawkReady = status['pixhawk']
        self.current_status.arduinoReady = status['arduino']
        self.current_status.hasGPSFix    = status['fix']
        self.current_status.hasWiFi      = status['wifi']

        self.pub_status.publish(self.current_status)
