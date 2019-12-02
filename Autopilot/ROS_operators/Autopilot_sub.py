#!/usr/bin/env python
'''
This class with a ROS publisher is used to subscribe and get data from behaviour topics

Questions: anhellesnes@fhs.mil.no
'''

import time
import rospy

from swarm.msg import Movement
from swarm.msg import Position
from Classes.GPS_class import GPS
from Classes.Vector_class import Vector

class swarmWanted():
    '''Helper class to fetch data from behaviour through ROS'''
    def __init__(self):
        '''Initialises subscribers to behaviour topics'''
        self.topic_main = "/swarm/behaviour/"
        
        topic_movement = "/swarm/behaviour/movement"
        topic_position = "/swarm/behaviour/position"
        try:
            rospy.Subscriber(topic_movement, Movement, self._update_movement)
            rospy.Subscriber(topic_position, Position, self._update_position)

        except rospy.exceptions.ROSException as e:
            print("could not subscribe to topic with error: {s}", format(e))
        
        self.swarm_movement = Vector()
        self.swarm_position = GPS()

        self.time_since = 0
        self.last_receive = rospy.get_rostime().secs
        self.newest = ''
        self.recieving_data = False

    def __call__(self):
        '''Call function that returns newest data

        Returns either:
            Vector of newest movement wanted
            GPS point wanted
        '''
        if self.newest == 'movement':
            return self.swarm_movement

        else:
            return self.swarm_position

    def _update_movement(self, data):
        self.swarm_movement.set(data.velocity, data.bearing)

        self.last_receive = rospy.get_rostime().secs
        self.newest = 'movement'

    def _update_position(self, data):
        self.swarm_position.set(data.latitude, data.longitude)

        self.last_receive = rospy.get_rostime().secs
        self.newest = 'position'

    def is_recieving(self):
        '''Helper function to return if topics are published to

        Returns:
            Boolean on wether its receiving or not
        '''
        self.time_since = rospy.get_rostime().secs-self.last_receive
        if self.time_since < 2 and self.newest != '':
            self.recieving_data = True
        
        return self.recieving_data