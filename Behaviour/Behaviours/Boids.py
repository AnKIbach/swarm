#!/usr/bin/env python
import numpy as np
import math as m

from Classes.GPS_class import GPS
from Classes.Vector_class import Vector

class boidBehavior():
    ''' Object for running calculations of Boids behaviour '''
    def __init__(self):

        self.Ka = 10.0
        self.Kc = 10.0
        self.Ks = 10.0
        self.tick = 0

        self.maxForce   = 0.9 # Magnitude of cohesion and separation
        self.maxSpeed   = 2.0 # Maximum speed in m/s
        self.perception = 100.0 # Max distance to ...

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
        # if self.movement.velocity > self.maxSpeed: #adjusting current speed to not exceed max
        #     self.movement.velocity = self.maxSpeed * 0.7

        alignment = self._calculate_alignment(global_list)
        cohesion = self._calculate_cohesion(global_list)
        separation = self._calculate_separation(global_list)

        print("alignment: ", alignment.showVector())
        print("cohesion: ", cohesion.showVector())
        print("separation: ", separation.showVector())

        wantedXY = alignment * self.Ka + cohesion * self.Kc + separation * self.Ks

        self.tick +=1
        while self.tick < 100:
            wantedXY.set(0.0, 1.0)
            self.tick += 1
        print("tid:", self.tick)
        # print("wantedXY: ")
        # wantedXY.showVector()
        return wantedXY

    def _handle_current(self, current_movement, current_position):
        self.position = current_position
        self.movement = current_movement

        self.has_newCurr = True

    def _calculate_cohesion(self, boats):
        cohesion = Vector()
        total = 0.0
        center_of_mass = Vector()
        for boid in boats:
            if boid['distance'] < self.perception and boid['distance'] != 0.0:
                center_of_mass.magnitude += boid['x']
                center_of_mass.angle     += boid['y']
                total += 1.0
        if total > 0.0 and center_of_mass.magnitude != 0.0 and center_of_mass.angle != 0.0:
            # center_of_mass = center_of_mass / total
            # center_of_mass = Vector(*center_of_mass)
            cohesion = center_of_mass.__truediv__(total) #- self.position
            cohesion_tot = m.sqrt(m.pow(cohesion.magnitude, 2.0)+m.pow(cohesion.angle, 2.0))

            if cohesion_tot > 0.0: #Makes the vector wanted in proportion with maxSpeed
                cohesion = (cohesion.__truediv__(cohesion_tot)) * self.maxSpeed
            if cohesion_tot > self.maxForce:
                cohesion = (cohesion.__truediv__(cohesion_tot)) * self.maxForce

        return cohesion # vector dowards center of mass

    def _calculate_separation(self, boats):
        separation = Vector()
        total = 0.0
        average_vector = Vector()
        for boid in boats:
            if boid['distance'] < self.perception and boid['distance'] != 0.0:
                diff = Vector(-boid['x'], -boid['y'])
                diff = diff.__truediv__(boid['distance'])
                average_vector += diff
                total += 1.0
        if total > 0.0 and average_vector.magnitude != 0.0 and average_vector.angle != 0.0:
            # average_vector = average_vector / total            
            separation = average_vector.__truediv__(total)#- self.movement
            separation_tot = m.sqrt(m.pow(separation.magnitude, 2.0)+m.pow(separation.angle, 2.0))

            print("separation: {} : {} tot: {}", separation.magnitude, separation.angle, separation_tot)

            if separation_tot > 0.0:
                separation = (separation.__truediv__(separation_tot)) * self.maxSpeed
            if separation_tot > self.maxForce:
                separation = (separation.__truediv__(separation_tot)) * self.maxForce

        return separation

    def _calculate_alignment(self, boats): #maybe working
        alignment = Vector()
        total = 0.0
        average_temp = Vector(0.0, 0.0)
        average_vector = Vector(0.0, 0.0)
        for boid in boats:
            if boid['distance'] < self.perception and boid['distance'] != 0.0: #finds number of boids within perception

                dx = boid['speed'] * m.sin(boid['bearing'])
                dy = boid['speed'] * m.cos(boid['bearing'])
                average_temp.set(dx,dy)

                average_vector += average_temp
                total += 1.0

        if total > 0.0 and average_vector.magnitude != 0.0 and average_vector.angle != 0.0:
            average_vector = average_vector.__truediv__(total)

            alignment_tot = m.sqrt(m.pow(alignment.magnitude, 2.0) + m.pow(alignment.angle, 2.0))

            if alignment_tot > 0.000:
                average_vector = average_vector * self.maxSpeed

            if alignment_tot > self.maxForce:
                alignment = (alignment.__truediv__(alignment_tot)) * self.maxForce

        return alignment