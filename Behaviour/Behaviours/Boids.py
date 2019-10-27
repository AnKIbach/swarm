import numpy as np
import math as m

from Classes.GPS_class import GPS
from Classes.Vector_class import Vector

class boidBehavior():

    def __init__(self, borders = []):
        
        self.maxForce   = 0.3 # Magnitude of cohesion and separation
        self.maxSpeed   = 2.0 # Maximum speed in m/s
        self.perception = 100 # Max distance to ...

        self.position = GPS()
        self.movement = Vector()
        # self.velocity = current_movement
        self.borders  = borders
        # self.boats    = []
        self.acceleration = Vector() # ncertain of usefulnes in code

        self.has_newCurr = False
        

    def __call__(self, position, movement, global_list):

        self._handle_current(position, movement)
        self._handle_borders()
        self._vect_to_xy(movement)

    
        if self.movement.velocity > self.maxSpeed: #adjusting current speed to not exceed max
            self.movement.velocity = self.maxSpeed * 0.7
        self.acceleration.set(0,0)

        #add handles of current and boats here

        alignment = self._calculate_alignment(global_list)
        cohesion = self._calculate_cohesion(global_list)
        separation = self._calculate_separation(global_list)

        self.acceleration += alignment + cohesion + separation #force adding

    def _handle_current(self, current_movement, current_position):
        self.position = current_position
        self.movement = current_movement

        self.has_newCurr = True

    def _handle_borders(self): #fix
        pass
        # if self.position > self.borders[]:
        #     self.position.lat = 0
        # elif self.position.lat < 0:
        #     self.position.lat = self.borders[]
        # if self.position.lon > self.borders[]:
        #     self.position.lon = 0
        # elif self.position.lon < 0:
        #     self.position.lon = self.borders[]

    def _vect_to_xy(self, movement):
        pass

    def _calculate_cohesion(self, boats):
        steering = Vector() 
        total = 0
        center_of_mass = Vector()
        for boid in boats:
            if boid['distance'] < self.perception:
                center_of_mass += boid.position
                total += 1
        if total > 0:
            center_of_mass /= total
            center_of_mass = Vector(*center_of_mass)
            vec_to_com = center_of_mass - self.position
            if np.linalg.norm(vec_to_com) > 0:
                vec_to_com = (vec_to_com / np.linalg.norm(vec_to_com)) * self.maxSpeed
            steering = vec_to_com - self.velocity
            if np.linalg.norm(steering)> self.maxForce:
                steering = (steering /np.linalg.norm(steering)) * self.maxForce
        return steering

    def _calculate_separation(self, boats):
        steering = Vector(0,0)
        total = 0
        average_vector = Vector(0,0)
        for boid in boats:
            distance = np.linalg.norm(boid.position - self.position)
            if self.position != boid.position and distance < self.perception:
                diff = self.position - boid.position
                diff /= distance
                average_vector += diff
                total += 1
        if total > 0:
            average_vector = Vector(*average_vector/total)
            if np.linalg.norm(steering) > 0:
                average_vector = (average_vector / np.linalg.norm(steering)) * self.maxSpeed
            steering = average_vector - self.velocity
            if np.linalg.norm(steering) > self.maxForce:
                steering = (steering /np.linalg.norm(steering)) * self.maxForce
        return steering

    def _calculate_alignment(self, boats): #maybe working
        steering = Vector()
        total = 0
        total_vector = Vector()
        for boid in boats:
            if boid['distance'] < self.perception: #finds number of boids within perception
                total_vector.magnitude += boid['speed']
                total_vector.angle += boid['bearing']
                total += 1
        if total > 0:
            average_vector = total_vector / total #uncertain of value
            average_vector = (average_vector / np.linalg.norm(average_vector)) * self.maxSpeed
            steering = average_vector - self.velocity
        return steering 