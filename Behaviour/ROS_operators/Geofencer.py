import rospy

from swarm.msg import SwarmHeader
from swarm.msg import SwarmCommand
from swarm.msg import Position

BOATS_IN_SWARM = 10  

class swarmData:
    def __init__(self):
        self.fence = [Position(), Position()]

        rospy.init_node('behaviour', anonymous=True)

        topic_command = "/swarm/com/command"

        rospy.Subscriber(topic_command, SwarmCommand, self._update_fence)
    
    def __call__(self):
        return self.fence

    def _update_fence(self, data):
        self.fence = data.destination

