#!/usr/bin/env python
import random
import math as m

from Classes.GPS_class import GPS
from Classes.Vector_class import Vector

class psoBehaviour():
    '''Calcualtion of PSO behaviour based on own and other boats positions in swarm'''
    def __init__(self, fence, posWanted):
        self.K1 = 1.0
        self.K2 = 1.0
        self.Kr = 1.0

        self.maxForce   = 0.3 # Magnitude of cohesion and separation - not used by now
        self.maxDist    = 100 # Variable for weighting distances
        self.maxSpeed   = 2.0 # Maximum speed in m/s
        self.perception = 100 # Max distance to ...

        self.fence = fence
        self.wanted     = posWanted
        self.best_self   = {'position': GPS(), 'value':0.0}
        self.best_global = {'position': GPS(), 'value':0.0}

        self.position = GPS()
        self.movement = Vector()

        self.has_newCurr = False

    def __call__(self, position, movement, global_list):
        self._handle_current(movement, position)

        self.fitness(global_list)

        wanted = self._calculate()
        return wanted

    def _handle_current(self, current_movement, current_position):
        self.movement = current_movement
        self.position = current_position

        self.has_newCurr = True

    def fitness(self, boats):
        self._check_pBest(self.position)
        self._check_gBest(boats)

    def _check_gBest(self, boats):
        boatPos = GPS()
        for boat in boats:
            boatPos.set(boat['lat'], boat['lon'])
            distCent = boatPos.calculate(self.wanted)
            boatVal = self.noise_function(distCent)
            if boatVal > self.best_global['value']:
                self.best_global['position'] = boatPos
                self.best_global['value']    = boatVal

    def _check_pBest(self, position):
        dist = position.calculate(self.wanted)
        current_self = self.noise_function(dist)
        if current_self > self.best_self['value']:
            self.best_self['position'] = self.position
            self.best_self['value']    = current_self

    def noise_function(self, distance):
        noise = random.randrange(0, 100) /100
        value = noise + (self.perception/(m.pow(distance,2))) # function 1/r^2 with noise and perception
        return value

    def _calculate(self):
        pbest = Vector()
        gbest = Vector()
        curr  = self.movement
        rand  = Vector()

        vec_pbest = self.position.calculate(self.best_self['position'])
        if vec_pbest.magnitude > self.maxSpeed:
            vec_pbest.magnitude = (vec_pbest.magnitude / self.maxDist) * self.maxSpeed

        vec_gbest = self.position.calculate(self.best_global['position'])
        if vec_gbest.magnitude > self.maxSpeed:
            vec_gbest.magnitude = (vec_gbest.magnitude / self.maxDist) * self.maxSpeed

        rand.magnitude = random.randrange(0,10)/10 * self.maxSpeed
        rand.angle     = random.randrange(0,360)

        tot = curr + pbest * self.K1 + gbest * self.K2 + rand * self.Kr

        print(tot)
        return tot