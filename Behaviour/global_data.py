
import rospy

from autopilot.msg import SwarmHeader
from autopilot.msg import BoatOdometry
from autopilot.msg import Position
from autopilot.msg import Movement

BOATS_IN_SWARM = 5

class swarmData:
    def __init__(self):
        self.list_global = []

        for i in range(BOATS_IN_SWARM):
            self.list_global.append(BoatOdometry())
        

        rospy.init_node('behaviour', anonymous=True)

        topic_odometry = "/swarm/data/odom"

        rospy.Subscriber(topic_odometry, BoatOdometry, self._update)
    
    def __call__(self):
        return self.list_global

    def _update(self, data):
        ID = data.header.id

        self.list_global[ID] = data

        self._get_time_since(data.header,ID)

    def _get_time_since(self, header, id): #for use of Ack from GCS
        pass

        
