#!/usr/bin/env python
import random
import math as m

from Classes.GPS_class import GPS
from Classes.Vector_class import Vector

class psoBehaviour():
    '''Calcualtion of PSO behaviour based on own and other boats positions in swarm'''
    def __init__(self, fence, posWanted):
        self.K1 = 0.5
        self.K2 = 0.5
        self.Kr = 1.0

        self.maxForce   = 0.8 # Magnitude of cohesion and separation - not used by now
        self.maxDist    = 100.0 # Variable for weighting distances
        self.minDist    = 1.0
        self.maxSpeed   = 2.0 # Maximum speed in m/s
        self.perception = 100.0 # Max distance to ...

        self.fence      = fence
        self.wanted     = posWanted
        self.best_self   = {'position': GPS(), 'value':0.0}
        self.best_global = {'position': GPS(), 'value':0.0}

        self.position = GPS()
        self.movement = Vector()

        self.has_newCurr = False

    def __call__(self, position, movement, global_list):
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
        boatPos = GPS()
        for boat in boats:
            boatPos.set(boat['lat'], boat['lon'])
            distCent = boatPos.calculate(self.wanted)
            boatVal = self.noise_function(distCent.magnitude)
            if boatVal > self.best_global['value']:
                self.best_global['position'] = boatPos
                self.best_global['value']    = boatVal

        print("gbest: ", self.best_global['value'])

    def _check_pBest(self):
        self.wanted.show()
        dist = self.position.calculate(self.wanted)
        current_self = self.noise_function(dist.magnitude)
        if current_self > self.best_self['value']:
            self.best_self['position'] = self.position
            self.best_self['value']    = current_self

    def noise_function(self, distance):
        print("distance", distance)
        if distance != 0.0:
            noise = random.randrange(1, 100) /100
            value = float(noise) + (self.perception / m.pow(distance, 2.0)) # function 1/r^2 with noise and perception
            return value
        else:
            return 0.0

    def _calculate(self, boats):
        boatPos = GPS()
        pbest = Vector()
        gbest = Vector()
        curr  = self.movement
        rand  = Vector()
        sep   = Vector()

        for boat in boats:
            boatPos.set(boat['lat'], boat['lon'])
            dist = self.position.calculate(boatPos)
            if dist > self.minDist:
                sep.set(1.0, (dist.angle - 180.0))

        vec_pbest = self.position.calculate(self.best_self['position'])
        if vec_pbest.magnitude > self.maxSpeed:
            vec_pbest.magnitude = (vec_pbest.magnitude / self.maxDist) * self.maxSpeed
            print("vector to pbest: ")
            vec_pbest.showVector()

        vec_gbest = self.position.calculate(self.best_global['position'])
        if vec_gbest.magnitude > self.maxSpeed:
            vec_gbest.magnitude = (vec_gbest.magnitude / self.maxDist) * self.maxSpeed
            print("vector to gbest: ")
            vec_gbest.showVector()

        # rand.magnitude = random.randrange(0,10) / 10
        # rand.angle     = random.randrange(0,360)

        tot = curr + pbest * self.K1 + gbest * self.K2 + sep * self.Kr

        print("tot:")
        tot.showVector()
        return tot