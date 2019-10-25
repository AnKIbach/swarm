#!/usr/bin/env python
import time
import numpy as np
import math as m

from enum import IntEnum

from Behaviours.Classes.GPS_class import GPS
from Behaviours.Classes.Vector_class import Vector

from Behaviours.Boids import boidBehavior
from Behaviours.PSO import waypointBehaviour

from autopilot.msg import BoatOdometry

class BehaviourType(IntEnum):
    BOID     = 0
    PSO      = 1
    OTHER    = 2
    SPECIALE = 3


class Behave: # funny :)
    def __init__(self, ID, use_behaviour = 0):
        self.current_position = GPS()
        self.boat_id = ID

        self._handle_behaviour(use_behaviour)
        
    def __call__(self, current, global_list): # both is boatodom obejcts - g_l is list of those

        self.current_position.set(current.position.lat, current.position.lon)

        if self.behaviour == "BOIDS":
            self.distances = self._calculate_distances(current['position'], global_list)
            self._make_list(self.distances, global_list['movement']) 
            
            self.behaviour(current.position, current.movement, self.distances)


    def _handle_behaviour(self, behaviour):   

        if behaviour == BehaviourType.BOID:
            self.behaviour_chosen = "BOIDS"
            self.behaviour = boidBehavior() #add borders
        if behaviour == BehaviourType.PSO:
            self.behaviour_chosen = "PSO"

    def _calculate_distances(self, current, global_list):
        other = GPS()
        distances = [] 
        
        for i in global_list:
            if self.boat_id == i: #to not calculate distance to self
                pass
            else:
                other.set(global_list[i].position.latitude, global_list[i].position.longitude)
                distance = self.current_position.calculate(other)

                distances.append(distance) #list of vectors

        return distances

    def _make_list(self, distances, velocities, bearings):
        pass



