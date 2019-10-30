import rospy

from swarm.msg import SwarmHeader
from swarm.msg import BoatOdometry

BOATS_IN_SWARM = 4  

class swarmData:
    def __init__(self):
        self.list_global = [BoatOdometry()] * (BOATS_IN_SWARM + 1)


        rospy.init_node('behaviour', anonymous=True)

        topic_odometry = "/swarm/data/odom"

        rospy.Subscriber(topic_odometry, BoatOdometry, self._update)
    
    def __call__(self):
        return self.list_global

    def _update(self, data):
        try:
            ID = data.header.id

            self.list_global[ID] = data

            self._get_time_since(data.header, ID)
        except:
            pass

    def _get_time_since(self, header, id): #for use of Ack from GCS
        pass

        
class swarmOut():
    def __init__(self):
        pass