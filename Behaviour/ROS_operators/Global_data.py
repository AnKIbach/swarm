import rospy

# from swarm.msg import SwarmHeader
from swarm.msg import BoatOdometry

BOATS_IN_SWARM = 10 

class swarmData:
    '''Helper class to continously keep an updated list of other boats'''
    def __init__(self):
        '''Initialises subscriber to data from multicaster'''
        self.list_global = [BoatOdometry()] * (BOATS_IN_SWARM)

        rospy.init_node('behaviour', anonymous=True)

        topic_odometry = "/swarm/data/odom"

        rospy.Subscriber(topic_odometry, BoatOdometry, self._update)
    
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

            self._get_time_since(data.header, ID)

        except IndexError:
            pass

    def _get_time_since(self, header, id): #for use of Ack from GCS
        pass
