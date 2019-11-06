#!/usr/bin/env python
import time
import rospy
import math as m

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
    def __init__(self, ID, fencePOS, use_behaviour = 0):
        self.current_position = GPS()
        self.current_movement = Vector()
        self.boat_id = ID
        self.fence_center = fencePOS #could insert GPS point here for test
        self.fence_radius = 75.0

        self._handle_behaviour(use_behaviour)

        self.has_newSelf = False
        self.inside_fence = True
        
    def __call__(self, global_list): # g_l is list of boatodom
        self._update_current(global_list)
        toFence = self._check_fence()
        if self.inside_fence == True:
            
            if self.behaviour_chosen == "BOIDS" and self.has_newSelf == True:
                behaviour_data = self._make_list(global_list) 

                self.has_newSelf = False
                behaviourXY = self.behaviour(self.current_position, self.current_movement, behaviour_data)
                
                return self._get_vec(behaviourXY)
        else:
            return toFence #Turns boat around if its outside the fence - probably wont work
        
    def _handle_behaviour(self, behaviour):   
        if behaviour == BehaviourType.BOID:
            self.behaviour_chosen = "BOIDS"
            self.behaviour = boidBehavior() #add borders
            
        if behaviour == BehaviourType.PSO:
            self.behaviour_chosen = "PSO"

    def _update_current(self, data):
        try:
            self.current_position.set(data[self.boat_id].position.latitude, data[self.boat_id].position.longitude)
            self.current_movement.set(data[self.boat_id].movement.velocity, data[self.boat_id].movement.bearing)

            print(self.current_movement.magnitude)

            self.has_newSelf = True

        except IndexError as e:
            self.has_newSelf = False
            print("could not update current position with erorr: {s}", format(e))

    def _make_list(self, dataObj):
        clist = [] 

        for i in range(len(dataObj)):
            if i == self.boat_id: #removes unwanted elements from list - i.e own boat and empty elements
                pass
            elif dataObj[i].position.latitude == 0.0 and dataObj[i].position.longitude == 0.0:
                pass
            else:
                dist = self._get_distance(dataObj[i].position)
                x, y = self._get_xy(dist)
                clist.append({"speed" : dataObj[i].movement.velocity,
                            "bearing" : dataObj[i].movement.bearing,
                            "distance": dist.magnitude,
                            "relative": dist.angle,
                            "x"       : x,
                            "y"       : y})
            
        print("clist: ", clist)
        return clist

    def _get_distance(self, pos):
        other = GPS()

        other.set(pos.latitude, pos.longitude)
        distance = self.current_position.calculate(other)

        return distance

    def _check_fence(self):
        try:
            distFence = self.current_position.calculate(self.fence_center)   
            if distFence.magnitude >= self.fence_radius:
                self.inside_fence = False
                return distFence
            else:
                self.inside_fence = True
                return 0.0
        except:
            pass
            # print("no fence recieved")

    def _get_xy(self, vector):
        dx = vector.magnitude * m.sin(vector.angle)
        dy = vector.magnitude * m.cos(vector.angle)

        return dx, dy

    def _get_vec(self, XY):
        vec = Vector()
        if XY.magnitude != 0.0 and XY.angle != 0.0:
            magnitude = m.sqrt(m.pow(XY.magnitude, 2) + m.pow(XY.angle, 2))
            angle = m.atan(XY.magnitude/XY.angle) #magn = x, angle = y

            vec.set(magnitude, angle)

        else:
            vec.set(0.0, 0.0)

        return vec