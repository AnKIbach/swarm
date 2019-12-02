#!/usr/bin/env python
'''
This class with a ROS subscriber is used to build a list of data from all units in swarm

Questions: anhellesnes@fhs.mil.no
'''

import rospy

from swarm.msg import BoatOdometry

BOATS_IN_SWARM = 10 #How many boats there can be max in swarm

class swarmData:
    '''Helper class to continously keep an updated list of other boats'''
    def __init__(self):
        '''Initialises subscriber to data from multicaster'''
        
        self.list_global = [BoatOdometry()] * (BOATS_IN_SWARM)

        rospy.init_node('behaviour', anonymous=True)

        topic_odometry = "/swarm/data/odom"

        rospy.Subscriber(topic_odometry, BoatOdometry, self._update)

        self.data_recieved = False
    
    def __call__(self):
        '''Caller function to return newest list of data

        Returns:
            List of ROS BoatOdometry msg objects for each boat in swarm
        '''
        return self.list_global

    def _update(self, data):

        try:
            ID = data.header.id

            self.list_global[ID] = data

            self.data_recieved = True

        except IndexError:
            pass

    def has_recieved(self): 
        if self.data_recieved == True:
            return True
        else:
            return False
