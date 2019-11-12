import rospy

# from swarm.msg import SwarmHeader
from swarm.msg import SwarmCommand
from swarm.msg import Movement
from swarm.msg import Position

class Subscriber:
    '''Reads, stores and forwards commands for swarm, reads from topic given'''
    def __init__(self):
        topic_command = "/swarm/com/command"

        rospy.Subscriber(topic_command, SwarmCommand, self._handle_command)

        self.command    = SwarmCommand()
        self.wanted_mov = Movement()
        self.wanted_pos = Position()
        self.fence      = Position()

        self.static_fence = Position()
        self.static_fence.latitude  = 60.365991
        self.static_fence.longitude = 5.264527

        self.new_command = False

    def _handle_command(self, command):
        self.command = command
        self._handle_specifics(command)

        self.new_command = True

    def _handle_specifics(self, data):
        self.wanted_mov.velocity = data.speed
        self.wanted_mov.bearing  = data.heading

        self.wanted_pos = data.destination
        self.fence      = data.fence

    def _update_fence(self, data):
        self.fence = data.destination

    def get_static_fence(self):
        return self.static_fence

    def get_command(self):
        return self.command

    def get_taskType(self):
        return self.command.taskType
    
    def get_headingMode(self):
        return self.command.headingMode

    def get_wantedMov(self):
        return self.wanted_mov
    
    def get_wantedPos(self):
        return self.wanted_pos

    def stop(self):
        return self.command.doImidiate

    def has_new(self):
        return self.new_command

    