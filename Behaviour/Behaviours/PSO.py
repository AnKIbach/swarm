#!/usr/bin/env python
import random
import math as m
import numpy as np

from Classes.GPS_class import GPS
from Classes.Vector_class import Vector

class boidBehavior():

    def __init__(self, posWanted):

        self.K1 = 1.0
        self.K2 = 1.0
        self.Kw = 1.0

        self.wanted     = posWanted
        self.maxForce   = 0.3 # Magnitude of cohesion and separation
        self.maxSpeed   = 2.0 # Maximum speed in m/s
        self.perception = 100 # Max distance to ...

        self.best_self   = {'position': GPS(), 'value':0.0}
        self.best_global = {'position': GPS(), 'value':0.0}

        self.position = GPS()
        self.movement = Vector()

        self.has_newCurr = False

    def __call__(self, position, movement, global_list):
        self._handle_current(position, movement)

        self.fitness(global_list)

    def _handle_current(self, current_movement, current_position):
        self.position = current_position
        self.movement = current_movement

        self.has_newCurr = True

    def fitness(self, boats):
        self._check_gBest(boats)
        self._check_pBest(self.position)

    def _check_gBest(self, boats):
        pass

    def _check_pBest(self, function):
        dist = self.position.calculate(self.wanted)
        current_self = self.noise_function(dist)
        if current_self > self.best_self:
            self.best_self['position'] = self.position
            self.best_self['value']    = current_self
        else:
            pass

    def noise_function(self, distance):
        noise = random.randrange(0 , 1.0, 0.01)
        value = noise * (self.perception/(m.pow(distance,2))) # function 1/r^2 with noise and perception
        return value


# Particle Swarm Optimization
def PSO(problem, MaxIter = 100, PopSize = 100, c1 = 1.4962, c2 = 1.4962, w = 0.7298, wdamp = 1.0):

    # Extract Problem Info
    CostFunction = problem['CostFunction'];
    VarMin = problem['VarMin'];
    VarMax = problem['VarMax'];
    nVar = problem['nVar'];

    # Initialize Global Best
    gbest = {'position': None, 'cost': np.inf};

    # Create Initial Population
    pop = [];
    for i in range(0, PopSize):
        pop.append(empty_particle.copy());
        pop[i]['position'] = np.random.uniform(VarMin, VarMax, nVar);
        pop[i]['velocity'] = np.zeros(nVar);
        pop[i]['cost'] = CostFunction(pop[i]['position']);
        pop[i]['best_position'] = pop[i]['position'].copy();
        pop[i]['best_cost'] = pop[i]['cost'];
        
        if pop[i]['best_cost'] < gbest['cost']:
            gbest['position'] = pop[i]['best_position'].copy();
            gbest['cost'] = pop[i]['best_cost'];
    
    # PSO Loop
    for it in range(0, MaxIter):
        for i in range(0, PopSize):
            
            pop[i]['velocity'] = w*pop[i]['velocity'] \
                + c1*np.random.rand(nVar)*(pop[i]['best_position'] - pop[i]['position']) \
                + c2*np.random.rand(nVar)*(gbest['position'] - pop[i]['position']);

            pop[i]['position'] += pop[i]['velocity'];
            pop[i]['position'] = np.maximum(pop[i]['position'], VarMin);
            pop[i]['position'] = np.minimum(pop[i]['position'], VarMax);

            pop[i]['cost'] = CostFunction(pop[i]['position']);
            
            if pop[i]['cost'] < pop[i]['best_cost']:
                pop[i]['best_position'] = pop[i]['position'].copy();
                pop[i]['best_cost'] = pop[i]['cost'];

                if pop[i]['best_cost'] < gbest['cost']:
                    gbest['position'] = pop[i]['best_position'].copy();
                    gbest['cost'] = pop[i]['best_cost'];

        w *= wdamp;
        print('Iteration {}: Best Cost = {}'.format(it, gbest['cost']));

    return gbest, pop;