#!/usr/bin/env python
import time
import rospy

from enum import IntEnum

from Behaviours.Classes.GPS_class import GPS
from Behaviours.Classes.Vector_class import Vector

from Behaviours.Boids import boidBehavior
# from Behaviours.PSO import waypointBehaviour

from swarm.msg import BoatOdometry

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
            
            behaviour_data = self._make_list(global_list) 

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

    def _make_list(self, dataObj):
        clist = [] 

        for i in range(len(dataObj)):
            if i == self.boat_id:
                pass
            else:
                dist = self._get_distance(dataObj[i].position)
                clist.append({"speed" : dataObj[i].movement.velocity,
                            "bearing" : dataObj[i].movement.bearing,
                            "distance": dist.magnitude,
                            "relative": dist.angle })

        return clist

    def _get_distance(self, pos):
        other = GPS()

        other.set(pos.latitude, pos.longitude)
        distance = self.current_position.calculate(other)

        return distance





