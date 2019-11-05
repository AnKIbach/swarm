import rospy

from swarm.msg import SwarmHeader
from swarm.msg import SwarmCommand
from swarm.msg import Position

class Fence:
    '''Reads and forwards fence for swarm, position gives centre for geofence circle'''
    def __init__(self):
        self.fence = [Position()]

        rospy.init_node('behaviour', anonymous=True)

        topic_command = "/swarm/com/command"

        rospy.Subscriber(topic_command, SwarmCommand, self._update_fence)
    
    def __call__(self):
        return self.fence

    def _update_fence(self, data):
        self.fence = data.destination

