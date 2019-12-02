#!/usr/bin/env python
'''
Autopilot_caller.py
This class is responsible for formating vector for PID-regulator in autopilot

Questions: anhellesnes@oslo.mil.no
'''

import time


from Classes.PID import PID
from Classes.GPS_class import GPS
from Classes.Vector_class import Vector

class Autopilot:
    def __init__(self, use_guidance = True):
        self.use_guidance = use_guidance

        self.guided_velocity = 0.0
        self.guided_angle = 0.0

        self.wanted_x = 0.0
        self.wanted_y = 0.0
        
        self.controller = PID()

    def set_wanted_xy(self, velocity_east, velocity_north):
        '''Set wanted values if vector is x, y based
        
        Args:
            velocity_east: float for x value of velocity vector
            velocity_north: float for y value of velocity vector
        '''
        self.wanted_x = velocity_east
        self.wanted_y = velocity_north
        wanted_list = [self.wanted_x, self.wanted_y]

        self.controller.set_wanted(wanted_list)

    def set_wanted_vector(self, wanted):
        '''Set wanted values if vector is Vector format
        
        Args:
           wanted: Vector object containing wanted movement
        '''
        self.guided_magnitude = wanted.magnitude
        self.guided_angle = wanted.angle

        self.controller.set_wanted(wanted)
    
    def __call__(self, current):
        if isinstance(current, Vector):
            pass

        if self.use_guidance:
            update = self.controller.update(current)
            return update

        else:
            #change vector type magnitude/angle to XY
            update = self.controller.update(current)
            return update
