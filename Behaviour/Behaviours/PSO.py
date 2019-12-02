#!/usr/bin/env python
'''
This class calculates vector for the PSO behaviour

needs:
    from self
        current movement and pos
    from others in swarm
        positions in lat, lon

Questions: anhellesnes@fhs.mil.no
'''

import random
import math as m

from .Classes.GPS_class import GPS
from .Classes.Vector_class import Vector

class psoBehaviour():
    '''Calcualtion of PSO behaviour based on own and other boats positions in swarm'''
    def __init__(self, fence, posWanted):
        self.Kc = 0.5 #force constant for current
        self.K1 = 0.6 #force constant for personal best
        self.K2 = 0.6 #force constant for global best
        self.Kr = 0.8 #force for random

        self.maxForce   = 0.8   # Magnitude of cohesion and separation - not used by now
        self.maxDist    = 50.0  # Variable for weighting distances
        self.minDist    = 1.5   # Minimum distance before forced separation
        self.maxSpeed   = 1.5   # Maximum speed in m/s
        self.perception = 50.0  # Max distance to percieve other units

        self.fence      = fence
        self.wanted     = posWanted
        self.best_self   = {'position': GPS(), 'value':0.0}
        self.best_global = {'position': GPS(), 'value':0.0}

        self.position = GPS()
        self.movement = Vector()

        self.has_newCurr = False

    def __call__(self, position, movement, global_list):
        ''' Caller function for behaviour calculations

        Args:  
            position: GPS point containng current position for self 
            movement: Vector containing current movement for self
            global_list: List of dictionaries containing other boats position lat,lon
        
        Returns:
            Vector for wanted movement based on PSO behaviour
        '''

        self._handle_current(movement, position)

        if self.has_newCurr == True:
            self.fitness(global_list)

            wanted = self._calculate(global_list)
            return wanted
        else:
            return Vector(0.0, 0.0)

    def _handle_current(self, current_movement, current_position):
        self.movement = current_movement
        self.position = current_position
        print("self: ")
        self.position.show()

        self.has_newCurr = True

    def fitness(self, boats):
        self._check_pBest()
        self._check_gBest(boats)

    def _check_gBest(self, boats):
        ''' Function to check for new global best

        Args:  
            boats: list of dictionaries containing position of boats
        '''

        boatPos = GPS()
        for boat in boats:
            boatPos.set(boat['lat'], boat['lon'])
            distCent = boatPos.calculate(self.wanted)
            boatVal = self.noise_function(distCent.magnitude)
            if boatVal > self.best_global['value']:
                self.best_global['position'] = boatPos
                self.best_global['value']    = boatVal

    def _check_pBest(self):
        ''' Function to check for new personal best

        Args:  
            boats: list of dictionaries containing position of boats
        '''
        dist = self.position.calculate(self.wanted)
        current_self = self.noise_function(dist.magnitude)
        if current_self > self.best_self['value']:
            self.best_self['position'] = self.position
            self.best_self['value']    = current_self

    def noise_function(self, distance):
        if distance != 0.0:
            noise = random.randrange(1, 100) /100
            value = float(noise) + (self.perception / m.pow(distance, 2.0)) # function 1/r^2 with noise and perception
            return value
        else:
            return 0.0

    def _calculate(self, boats):
        boatPos = GPS()
        vec_pbest = Vector()
        vec_gbest = Vector()
        curr  = self.movement
        rand  = Vector()
        sep   = Vector()
        sep_tot = Vector()

        for boat in boats:
            boatPos.set(boat['lat'], boat['lon'])
            dist = self.position.calculate(boatPos)
            if dist > self.minDist:
                sep.set(1.0, (dist.angle - 180.0))
                sep = self._get_xy(sep)
            sep_tot += sep

        vec_pbest = self.position.calculate(self.best_self['position'])
        if vec_pbest.magnitude > self.maxSpeed:
            vec_pbest.magnitude = (vec_pbest.magnitude / self.maxDist) * self.maxSpeed
            print("vector to pbest: ")
            vec_pbest.showVector()
        vec_pbest = self._get_xy(vec_pbest)

        vec_gbest = self.position.calculate(self.best_global['position'])
        if vec_gbest.magnitude > self.maxSpeed:
            vec_gbest.magnitude = (vec_gbest.magnitude / self.maxDist) * self.maxSpeed
            print("vector to gbest: ")
            vec_gbest.showVector()
        vec_gbest = self._get_xy(vec_gbest)

        rand.magnitude = random.randrange(0,10) / 10
        rand.angle     = random.randrange(0,360)
        rand = self._get_xy(rand)
        
        if sep_tot.magnitude > 0.0 != sep_tot.angle != 0.0:
            tot = sep * self.Kr

        else:
            tot = curr * self.Kc + vec_pbest * self.K1 + vec_gbest * self.K2
            
            tot_tot = m.sqrt(m.pow(tot.magnitude, 2.0)+m.pow(tot.angle, 2.0))
            if tot_tot > self.maxSpeed:
                tot = (tot.__truediv__(tot_tot)) * self.maxSpeed

        print("tot:")
        tot.showVector()
        return tot

    def _get_xy(self, vector):
        dx = round(vector.magnitude * m.sin(m.radians(vector.angle)), 5)
        dy = round(vector.magnitude * m.cos(m.radians(vector.angle)), 5)

        return Vector(dx, dy)