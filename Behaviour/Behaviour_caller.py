#!/usr/bin/env python
import time
import numpy as np
import math as m

from enum import IntEnum

from Behaviours.Classes.GPS_class import GPS
from Behaviours.Classes.Vector_class import Vector

from Behaviours.Boids import boidBehavior
# from Behaviours.PSO import waypointBehaviour

from autopilot.msg import BoatOdometry

class BehaviourType(IntEnum):
    BOID     = 0
    PSO      = 1
    OTHER    = 2
    SPECIALE = 3


class Behave: # funny :)
    def __init__(self, ID, use_behaviour = 0):
        self.current_position = GPS()
        self.current_movement = Vector()
        self.boat_id = ID
        self.behaviour_data = []
        
        self._handle_behaviour(use_behaviour)

        self.has_newSelf = False
        
    def __call__(self, global_list): # g_l is list of boatodom
        self._update_current(global_list)

        if self.behaviour == "BOIDS" and self.has_newSelf == True:
            distances      = self._get_distances(global_list)
            self.behaviour_data = self._make_list(distances, global_list['movement']) 
            
            #self.behaviour(self.current_position, self.current_movement, self.behaviour_data)
            
        
        self.has_newSelf = False
        
        return self.behaviour_data

    def _update_current(self, global_list):
        self.current_position.set(global_list[self.boat_id].position.latitude, global_list[self.boat_id].position.longitude)
        self.current_movement.set(global_list[self.boat_id].movement.velocity, global_list[self.boat_id].movement.bearing)
        
        self.has_newSelf = True


    def _handle_behaviour(self, behaviour):   

        if behaviour == BehaviourType.BOID:
            self.behaviour_chosen = "BOIDS"
            self.behaviour = boidBehavior() #add borders
        if behaviour == BehaviourType.PSO:
            self.behaviour_chosen = "PSO"

    def _get_distances(self, global_list):
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

    def _make_list(self, distances, movements):
        clist = [{ "speed" : 0.0,
                "bearing": 0.0,
                "distance" : 0.0,
                "relative" : 0.0
                }] * len(distances)

        del movements[self.boat_id] #remove own speed and bearing

        for i in distances:
            clist[i] = {"speed"   : movements.velocity,
                        "bearing" : movements.bearing,
                        "distance": distances[i].magnitude,
                        "relative": distances[i].angle }

        return clist



