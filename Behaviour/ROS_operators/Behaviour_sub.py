#!/usr/bin/env python
'''
This class with a ROS subscriber handles new commands from base

Questions: anhellesnes@fhs.mil.no
'''

import rospy

# from swarm.msg import SwarmHeader
from swarm.msg import SwarmCommand
from swarm.msg import Movement
from swarm.msg import Position

class NewCommand(Exception):
    '''New command recieved, needs to be handled'''
    def __init__(self, msg): 
        self.msg = msg

    def __str__(self): 
        return(self.msg) 

class Subscriber:
    '''Reads, stores and forwards commands for swarm, reads from topic given'''
    def __init__(self):
        '''Initialises subscriber and other data for behaviour'''
        topic_command = "/swarm/com/command"

        rospy.Subscriber(topic_command, SwarmCommand, self._handle_command)

        self.command    = SwarmCommand()
        self.wanted_mov = Movement()
        self.wanted_pos = Position()
        self.fence      = Position()

        self.static_fence = Position()
        self.static_fence.latitude  = 60.3678
        self.static_fence.longitude = 5.2536

        self.new_command = False

    def __call__(self):
        if self.has_new():
            self.new_command = False
            raise NewCommand('t')

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
        '''Helper function to get static fence position

        Returns:
            GPS point for static fence center
        '''
        return self.static_fence

    def get_fence(self):
        '''Helper function to get static fence position

        Returns:
            GPS point for static fence center
        '''
        return self.fence

    def get_command(self):
        '''Helper function to get newest command recieved

        Returns:
            ROS msg object containing command from base
        '''
        return self.command

    def get_taskType(self):
        '''Helper function to get task type from command

        Returns:
            Integer containing task type from command
        '''
        return self.command.taskType

    def get_colavMode(self):
        '''Helper function to get colav mode from command

        Returns:
            Integer containing colav mode from command
        '''
        return self.command.colavMode
    
    def get_headingMode(self):
        '''Helper function to get heading mode from command

        Returns:
            Integer containing heading mode from command
        '''
        return self.command.headingMode

    def get_wantedMov(self):
        '''Helper function to get wanted movement from command

        Returns:
            ROS Movement msg containing wanted vector from command
        '''
        return self.wanted_mov
    
    def get_wantedPos(self):
        '''Helper function to get wanted position from command

        Returns:
            ROS Position msg containing wanted GPS point from command
        '''
        return self.wanted_pos

    def stop(self):
        '''Helper function to determine if base wants boat to stop

        Returns:
            Boolaen value if boat should stop or not
        '''
        return self.command.doImidiate

    def has_new(self):
        '''Helper function to check if boat has recieved new command

        Returns:
            Boolaen value if new command is recieved or not
        '''
        return self.new_command

    