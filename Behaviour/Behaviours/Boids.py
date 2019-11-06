#!/usr/bin/env python
import numpy as np
import math as m

from Classes.GPS_class import GPS
from Classes.Vector_class import Vector

class boidBehavior():

    def __init__(self):

        self.Ka = 1.0
        self.Kc = 1.0
        self.Ks = 1.0

        self.maxForce   = 0.3 # Magnitude of cohesion and separation
        self.maxSpeed   = 2.0 # Maximum speed in m/s
        self.perception = 100 # Max distance to ...

        self.position = GPS()
        self.movement = Vector()

        self.has_newCurr = False

    def __call__(self, position, movement, global_list):

        self._handle_current(position, movement)
        # if self.movement.velocity > self.maxSpeed: #adjusting current speed to not exceed max
        #     self.movement.velocity = self.maxSpeed * 0.7

        alignment = self._calculate_alignment(global_list)
        cohesion = self._calculate_cohesion(global_list)
        separation = self._calculate_separation(global_list)

        wantedXY = alignment * self.Ka + cohesion * self.Kc + separation * self.Ks

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
            print("com: ")
            center_of_mass.showVector()
            cohesion = center_of_mass / total #- self.position
            cohesion_tot = m.sqrt(m.pow(cohesion.magnitude, 2)+m.pow(cohesion.angle, 2))

            if cohesion_tot > 0.0: #Makes the vector wanted in proportion with maxSpeed
                cohesion = (cohesion / cohesion_tot) * self.maxSpeed

        return cohesion # vector dowards center of mass

    def _calculate_separation(self, boats):
        separation = Vector()
        total = 0.0
        average_vector = Vector()
        for boid in boats:
            if boid['distance'] < self.perception and boid['distance'] != 0.0:
                diff = Vector(-boid['x'], -boid['y'])
                diff /= boid['distance']
                average_vector += diff
                total += 1.0
        if total > 0.0 and average_vector.magnitude != 0.0 and average_vector.angle != 0.0:
            # average_vector = average_vector / total            
            print("com: ")
            average_vector.showVector()
            separation = average_vector / total#- self.movement
            separation_tot = m.sqrt(m.pow(separation.magnitude, 2)+m.pow(separation.angle, 2))

            if separation_tot > 0.0:
                separation = (separation / separation_tot) * self.maxForce
        return separation

    def _calculate_alignment(self, boats): #maybe working
        alignment = Vector()
        total = 0.0
        average_vector = Vector(0.0, 0.0)
        for boid in boats:
            if boid['distance'] < self.perception and boid['distance'] != 0.0: #finds number of boids within perception
                average_vector.magnitude += boid['speed']
                average_vector.angle     += boid['bearing']
                total += 1.0
        if total > 0.0 and average_vector.magnitude != 0.0 and average_vector.angle != 0.0:
            alignment.magnitude = average_vector.magnitude / total
            alignment.angle = average_vector.angle / total  

        return alignment