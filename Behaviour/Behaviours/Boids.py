#!/usr/bin/env python
'''
This class calculates vector for the Boids behaviour

needs:
    from self
        current movement and pos
    from others in swarm
        speed and bearing
        distance and relative angle
        distance to east (x)
        distance to north (x)

Questions: anhellesnes@fhs.mil.no
'''
import numpy as np
import math as m

from Classes.GPS_class import GPS
from Classes.Vector_class import Vector

class boidBehavior():
    ''' Object for running calculations of Boids behaviour '''
    def __init__(self):

        self.Ka = 1.5   #Adjusting alignment
        self.Kc = 1.85  #Adjusting cohesion
        self.Ks = 0.5   #Adjusting separation

        self.maxForce   = 1.2 # maximum magnitude of cohesion and separation
        self.maxSpeed   = 2.0 # Maximum speed in m/s
        self.perception = 100.0 # Max distance to percieve other boats

        self.position = GPS()
        self.movement = Vector()

        self.has_newCurr = False

    def __call__(self, position, movement, global_list):
        ''' Call function for initiated boid behaviour

        args:
            position: Vector of current position
            movement: Vector of current movement
            global_list: Pre-made list of dictionaries from caller with behaviour based
            data from other boats in swarm
        '''
        self._handle_current(position, movement)

        alignment = self._calculate_alignment(global_list)
        cohesion = self._calculate_cohesion(global_list)
        separation = self._calculate_separation(global_list)

        print"alignment x: ", alignment.magnitude * self.Ka, " y : ", alignment.angle * self.Ka
        print"cohesion x: ", cohesion.magnitude * self.Kc , " y : ", cohesion.angle * self.Kc
        print"separation x: ", separation.magnitude * self.Ks, " y : ", separation.angle * self.Ks

        wantedXY = alignment * self.Ka + cohesion * self.Kc + separation * self.Ks

        return wantedXY

    def _handle_current(self, current_movement, current_position):
        self.position = current_position
        self.movement = current_movement

        self.has_newCurr = True

    def _calculate_cohesion(self, boats):
        '''
        Function to calculate cohesion vector

        Args:
            boats: list of dictionaries containing data from other boats
        
        Returns:
            A vector containing cohesion for unit
        '''
        cohesion = Vector()
        total = 0.0
        center_of_mass = Vector()

        for boid in boats:
            if boid['distance'] < self.perception and boid['distance'] != 0.0: # only boids within perception calculated
                center_of_mass.magnitude += boid['x']
                center_of_mass.angle     += boid['y']
                total += 1.0

        if total > 0.0 and center_of_mass.magnitude != 0.0 and center_of_mass.angle != 0.0: #only manipulates vector if there is one
            cohesion = center_of_mass.__truediv__(total) #- self.position
            cohesion_tot = m.sqrt(m.pow(cohesion.magnitude, 2.0)+m.pow(cohesion.angle, 2.0))

            if cohesion_tot > 0.0: #Proportions the vector to maxSpeed
                cohesion = (cohesion.__truediv__(cohesion_tot)) * self.maxSpeed
            if cohesion_tot > self.maxForce:
                cohesion = (cohesion.__truediv__(cohesion_tot)) * self.maxForce

        return cohesion # vector towards center of mass - cohesion

    def _calculate_separation(self, boats):
        '''
        Function to calculate separation vector

        Args:
            boats: list of dictionaries containing data from other boats
        
        Returns:
            A vector containing separation for unit
        '''
        separation = Vector()
        total = 0.0
        average_vector = Vector()

        for boid in boats:
            if boid['distance'] < self.perception and boid['distance'] != 0.0: # only boids within perception calculated
                diff = Vector(-boid['x'], -boid['y'])
                diff = diff.__truediv__(boid['distance'])
                average_vector += diff
                total += 1.0

        if total > 0.0 and average_vector.magnitude != 0.0 and average_vector.angle != 0.0:          
            separation = average_vector.__truediv__(total)
            separation_tot = m.sqrt(m.pow(separation.magnitude, 2.0)+m.pow(separation.angle, 2.0))

            if separation_tot > 0.0: #Proportions the vector to maxSpeed
                separation = (separation.__truediv__(separation_tot)) * self.maxSpeed
            if separation_tot > self.maxForce:
                separation = (separation.__truediv__(separation_tot)) * self.maxForce

        return separation

    def _calculate_alignment(self, boats): 
        '''
        Function to calculate alignment vector

        Args:
            boats: list of dictionaries containing data from other boats
        
        Returns:
            A vector containing alignment for unit
        '''

        alignment = Vector()
        total = 0.0
        average_temp = Vector(0.0, 0.0)
        average_vector = Vector(0.0, 0.0)

        for boid in boats:
            if boid['distance'] < self.perception and boid['distance'] != 0.0: #finds number of boids within perception

                dx = boid['speed'] * m.sin(m.radians(boid['bearing'])) #converts vector to x,y components
                dy = boid['speed'] * m.cos(m.radians(boid['bearing']))
                average_temp.set(dx,dy)

                average_vector += average_temp
                total += 1.0

        if total > 0.0:
            try:
                alignment.magnitude = average_vector.magnitude / total
            except ValueError:
                pass

            try:
                alignment.angle = average_vector.angle / total
            except ValueError:
                pass

            alignment_tot = m.sqrt(m.pow(average_vector.magnitude, 2.0) + m.pow(average_vector.angle, 2.0))

            if alignment_tot > self.maxForce:
                alignment = (alignment.__truediv__(alignment_tot)) * self.maxForce

        return alignment