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

        rospy.Subscriber(topic_command, SwarmCommand, self._handle_command)

        self.command = SwarmCommand()

        self.new_command = False

    def __call__(self):
        return self.fence

    def _handle_command(self, command):
        pass

    def _update_fence(self, data):
        self.fence = data.destination

    def get_command(self):
        return self.command

    def get_taskType(self):
        return self.command.taskType
    
    def get_headingMode(self):
        return self.command.headingMode 

