#!/usr/bin/env python
import rospy

from swarm.msg import Movement
from swarm.msg import Position

class Talker:
    def __init__(self):
        topic_movement  = "/swarm/behaviour/movement"
        topic_position  = "/swarm/behaviour/position"

        self.pub_movement = rospy.Publisher(topic_movement, Movement, queue_size=10)
        self.pub_position = rospy.Publisher(topic_position, Movement, queue_size=10)
        
        self.wanted_movement = Movement()
        self.wanted_position = Position()

    def __call__(self, data, typ = 'movement'):
        
        # if there is need to use headers
        # self.time_now = rospy.get_rostime()
        # self.current_data.header.secs    = self.time_now.secs
        # self.current_data.header.nsecs   = self.time_now.nsecs
        # self.current_data.header.id      = BOAT_ID
        # self.current_data.header.msgType = 1

        # self.wanted_data.header = self.current_data.header

        if typ == 'movement': #probably pretty shady way to do it
            self.wanted_movement.velocity = data.magnitude
            self.wanted_movement.bearing  = data.angle
            
            self.pub_movement.publish(self.wanted_movement)

        elif typ == 'position': 
            self.wanted_position.latitude = data.latitude
            self.wanted_position.latitude = data.longidute

            self.pub_position.publish(self.wanted_position)

       
