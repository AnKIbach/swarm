#!/usr/bin/env python
import time

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

        self._handle_behaviour(use_behaviour)

        self.has_newSelf = False
        
    def __call__(self, global_list): # g_l is list of boatodom
        self._update_current(global_list)

        if self.behaviour_chosen == "BOIDS" and self.has_newSelf == True:
            distance_data  = self._get_distances(global_list)
            behaviour_data = self._make_list(distance_data, global_list) 

            self.has_newSelf = False
            #self.behaviour(self.current_position, self.current_movement, self.behaviour_data)
            return behaviour_data
        
    def _handle_behaviour(self, behaviour):   
        if behaviour == BehaviourType.BOID:
            self.behaviour_chosen = "BOIDS"
            # self.behaviour = boidBehavior() #add borders
            
        if behaviour == BehaviourType.PSO:
            self.behaviour_chosen = "PSO"

    def _update_current(self, data):
        self.current_position.set(data[self.boat_id].position.latitude, data[self.boat_id].position.longitude)
        self.current_movement.set(data[self.boat_id].movement.velocity, data[self.boat_id].movement.bearing)
        
        self.has_newSelf = True

    def _get_distances(self, data):
        other = GPS()
        dist = [] 
        
        for i in range(len(data)):
            if self.boat_id == i: #to not calculate distance to self
                pass
            else: 
                other.set(data[i].position.latitude, data[i].position.longitude)
                distance = self.current_position.calculate(other)

                dist.append(distance) #list of vectors

        return dist

    def _make_list(self, distances, movements):
        clist = [{ "speed" : 0.0,
                "bearing": 0.0,
                "distance" : 0.0,
                "relative" : 0.0
                }] * len(distances)

        del movements[self.boat_id] #remove own speed and bearing

        for i in range(len(distances)):
            clist[i] = ({"speed"   : movements[i].movement.velocity,
                        "bearing" : movements[i].movement.bearing,
                        "distance": distances[i].magnitude,
                        "relative": distances[i].angle })
        return clist



