#!/usr/bin/env python
'''
Behaviour_caller.py
This class is responsible for calling chosen behaviour and formatting data to behaviour

Questions: anhellesnes@fhs.mil.no
'''

import time
import rospy
import math as m

from enum import IntEnum

from Behaviours.Classes.GPS_class import GPS
from Behaviours.Classes.Vector_class import Vector

from Behaviours.Boids import boidBehavior
from Behaviours.PSO import psoBehaviour

from swarm.msg import BoatOdometry

class BehaviourType(IntEnum):
    BOID     = 0
    PSO      = 1
    OTHER    = 2
    SPECIALE = 3

class Behave:
    '''Class to run chosen behaviour'''
    def __init__(self, ID, fencePOS, use_behaviour = 0):
        '''Initializes data holders and chosen behaviour object'''
        
        self.current_position = GPS()
        self.current_movement = Vector()
        self.boat_id = ID
        self.fence_center = GPS(fencePOS.latitude, fencePOS.longitude) #could insert GPS point here for test
        self.fence_radius = 15.0

        self._handle_behaviour(use_behaviour)

        self.has_newSelf = False
        
    def __call__(self, global_list): 
        '''Caller function to check if unit is inside swarm and run chosen behaviour

        Args:
            global_list: list of data from boats from ROS

        Returns:
            Vector to wanted movement 
        '''
        self._update_current(global_list)
        toFence = self._check_fence()

        if toFence.magnitude <= self.fence_radius:
            if self.behaviour_chosen == "BOIDS" and self.has_newSelf == True:
                boid_data = self._make_list(global_list) 

                self.has_newSelf = False
                behaviourXY = self.boids(self.current_position, self.current_movement, boid_data)
                
                return self._get_vec(behaviourXY)
            
            if self.behaviour_chosen == "PSO" and self.has_newSelf == True:
                pso_data = self._make_PSO_list(global_list)

                self.has_newSelf = False

                vectorPSO = self.pso(self.current_position, self.current_movement, pso_data)

                return self._get_vec(vectorPSO)
        else:
            print"outside fence by ", toFence.magnitude, "m, returning to center "
            toFence.set(1.0, toFence.angle)
            return toFence 

    def change_fence(self, new_fence):
        '''Helper function to set new fence
        
        Args:
            GPS point containing center of new fence
        '''
        
        fence = GPS(new_fence.latitude, new_fence.longitude)
        self.fence_center = fence
    
    def set_destination(self, destination):
        '''Helper function to set a destination for behaviour
        
        Args:
            GPS point containing new destination
        '''
        destGPS = GPS(destination.latitude, destination.longitude)
        try: 
            del self.pso
            self.pso = psoBehaviour(self.fence_center, destGPS)

        except AttributeError as e:
            print"could not set destination, with error: ", e
        
    def _handle_behaviour(self, behaviour):   
        if behaviour == BehaviourType.BOID:
            self.behaviour_chosen = "BOIDS"
            self.boids = boidBehavior() #add borders
            
        if behaviour == BehaviourType.PSO:
            self.behaviour_chosen = "PSO"
            self.pso = psoBehaviour(self.fence_center, self.fence_center)

    def _update_current(self, data):
        try:
            self.current_position.set(data[self.boat_id].position.latitude, data[self.boat_id].position.longitude)
            self.current_movement.set(data[self.boat_id].movement.velocity, data[self.boat_id].movement.bearing)

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
        for i in range(len(clist)):
            print"data from boat ", i
            print(clist[i])
        print"elements in BOIDS list: ", len(clist)
        return clist

    def _make_PSO_list(self, dataObj):
        clist = [] 

        for i in range(len(dataObj)):
            if i == self.boat_id: #removes unwanted elements from list - i.e own boat and empty elements
                pass
            elif dataObj[i].position.latitude == 0.0 and dataObj[i].position.longitude == 0.0:
                pass
            else:
                # dist = self._get_distance(dataObj[i].position)
                clist.append({'lat' : dataObj[i].position.latitude,
                            'lon' : dataObj[i].position.longitude})

        print"elements in PSO list: ", len(clist)    
        return clist

    def _get_distance(self, pos):
        other = GPS()

        other.set(pos.latitude, pos.longitude)
        distance = self.current_position.calculate(other)

        return distance

    def _check_fence(self):
        try:
            distFence = self.current_position.calculate(self.fence_center)  
            
            return distFence

        except AttributeError as e:
            print(e)
            return Vector(0.0, 0.0)

    def _get_xy(self, vector):
        dx = round(vector.magnitude * m.sin(m.radians(vector.angle)), 5)
        dy = round(vector.magnitude * m.cos(m.radians(vector.angle)), 5)

        return dx, dy

    def _get_vec(self, XY):
        vec = Vector()
        if XY.magnitude != 0.0 and XY.angle != 0.0:
            magnitude = m.sqrt(m.pow(XY.magnitude, 2.0) + m.pow(XY.angle, 2.0))
            angle = m.degrees(m.atan2(XY.magnitude, XY.angle))

            if XY.magnitude < 0.0:
                angle = angle + 360    

            vec.set(magnitude, angle)
        else:
            vec = self.current_movement

        return vec

