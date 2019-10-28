import time
import rospy

from swarm.msg import Movement
from swarm.msg import Position

BOATS_IN_SWARM = 5

class swarmWanted():
    '''class responsible for fetching wanted movement from behaviour'''
    def __init__(self):
        self.topic_main = "/swram/behaviour"
        
        topic_movement = "/swarm/behaviour/movement"
        topic_position = "/swarm/behaviour/position"

        rospy.Subscriber(topic_movement, Movement, self._update_movement)
        rospy.Subscriber(topic_position, Movement, self._update_position)
        
        self.last_receive = rospy.get_rostime().secs
        self.newest = ''
        self.recieving_data = False

    def __call__(self):
        if self.newest == 'movement':
            return self.swarm_movement

        else:
            return self.swarm_position

    def _update_movement(self, data):
        self.swarm_movement = data

        self.last_receive = rospy.get_rostime()
        self.newest = 'movement'

    def _update_position(self, data):
        self.swarm_position = data

        self.last_receive = rospy.get_rostime()
        self.newest = 'position'

    def is_recieving(self):
        if (rospy.get_rostime().secs-self.last_receive) < 2.0 and self.newest != '':
            self.recieving_data = True
        
        return self.recieving_data