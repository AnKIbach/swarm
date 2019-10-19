#!/usr/bin/env python
import time
import numpy as np
import math as m

from Behaviours.Classes.GPS_class import GPS
from Behaviours.Classes.Vector_class import Vector

from Behaviours import Boids
from Behaviours import PSO

class Behave: # funny :)
    def __init__(self, use_behaviour = Boids):
        self._handle_behaviour(use_behaviour)

        self.guided_velocity = 0.0
        self.guided_angle = 0.0

        self.wanted_x = 0.0
        self.wanted_y = 0.0
        
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

    def _handle_behaviour(self, chosen_behaviour):
        pass #masse if setninger?

    def _calculate_distances(self, global_list):
        pass

    def _use_xy(self, velocity_east, velocity_north):
        self.wanted_x = velocity_east
        self.wanted_y = velocity_north
        wanted_list = [self.wanted_x, self.wanted_y]

        self.controller.set_wanted(wanted_list)

    def _use_vector(self, wanted):
        self.guided_magnitude = wanted.magnitude
        self.guided_angle = wanted.angle

        self.controller.set_wanted(wanted)
    


