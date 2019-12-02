#!/usr/bin/env python
'''
This class with a ROS publisher sends data on wanted to ROS

Questions: anhellesnes@fhs.mil.no
'''

import rospy

from swarm.msg import Movement
from swarm.msg import Position

class Talker:
    ''' Helper class to let behaviour publish wanted movement to ROS'''
    def __init__(self):
        '''Initialises ROS publishers'''
        topic_movement  = "/swarm/behaviour/movement"
        topic_position  = "/swarm/behaviour/position"

        self.pub_movement = rospy.Publisher(topic_movement, Movement, queue_size=10)
        self.pub_position = rospy.Publisher(topic_position, Position, queue_size=10)
        
        self.wanted_movement = Movement()
        self.wanted_position = Position()

    def __call__(self, data, typ = 'movement'):
        '''Caller function to publish newest data

        Args:
            data: Either a Vector or GPS point containing wanted from behaviour
        '''

        if typ == 'movement': #probably pretty shady way to do it
            self.wanted_movement.velocity = data.magnitude
            self.wanted_movement.bearing  = data.angle
            
            self.pub_movement.publish(self.wanted_movement)

        elif typ == 'position': 
            self.wanted_position.latitude = data.latitude
            self.wanted_position.latitude = data.longidute

            self.pub_position.publish(self.wanted_position)

       
