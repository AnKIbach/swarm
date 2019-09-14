#!/usr/bin/env python
import time

from PID import PID
import PID_plotter as plot
from Vector_definition import Vector

class Autopilot:
    def __init__(self, use_guidance = True):
        self.use_guidance = use_guidance

        self.guided_velocity = 0.0
        self.guided_angle = 0.0

        self.wanted_x = 0.0
        self.wanted_y = 0.0
        
        self.controller = PID()

    def set_wanted_xy(self, velocity_east, velocity_north):
        self.wanted_x = velocity_east
        self.wanted_y = velocity_north
        wanted_list = [self.wanted_x, self.wanted_y]

        self.controller.set_wanted(wanted_list)

    def set_wanted_vector(self, wanted):
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
    

    def show_history(self, *args):
        ply = plot.plotter()
        ply.present(args)
        

