import numpy as np
import math as m

from Classes.GPS_class import GPS
from Classes.Vector_class import Vector

class boidBehavior():

    def __init__(self, borders = []):
        
        self.maxForce   = 0.3 # Magnitude of cohesion and separation
        self.maxSpeed   = 2.0 # Maximum speed in m/s
        self.perception = 200 # Max distance to ...

        self.position = GPS()
        # self.velocity = current_movement
        self.borders  = borders
        self.boats    = []
        self.acceleration = Vector(0.0, 0.0) # ncertain of usefulnes in code
        

    def __call__(self, current_position, current_movement, global_list):
        self.position += self.velocity
        self.velocity += self.acceleration
    
        if np.linalg.norm(self.velocity) > self.maxSpeed:
            self.velocity = self.velocity / np.linalg.norm(self.velocity) * self.maxSpeed
        self.acceleration = Vector(0,0)

        #add handles of current and boats here

        alignment = self._calculate_alignment(current_movement, global_list)
        cohesion = self._calculate_cohesion(current_movement, global_list)
        separation = self._calculate_separation(current_movement, global_list)

        self.acceleration += alignment + cohesion + separation #force adding

    def _handle_current(self, current_movement, current_position):
        pass

    def _handle_borders(self, borders): #fix
        if self.position.lat > self.borders[x]:
            self.position.lat = 0
        elif self.position.lat < 0:
            self.position.lat = self.borders[x]
        if self.position.lon > self.borders[y]:
            self.position.lon = 0
        elif self.position.lon < 0:
            self.position.lon = self.borders[y]

    def _vector_to_xy(self, velocity, bearing):
        pass

    def _calculate_alignment(self, current, boats):
        steering = Vector(0,0)
        total = 0
        average_vector = Vector(0,0)
        for boid in boats:
            if np.linalg.norm(boid.position - self.position) < self.perception:
                average_vector += boid.velocity
                total += 1
        if total > 0:
            average_vector = Vector(*average_vector/total)
            average_vector = (average_vector / np.linalg.norm(average_vector)) * self.maxSpeed
            steering = average_vector - self.velocity
        return steering 

    def _calculate_cohesion(self, current, boats):
        steering = Vector(0,0) 
        total = 0
        center_of_mass = Vector(0,0)
        for boid in boats:
            if  np.linalg.norm(boid.position - self.position) < self.perception:
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

    def _calculate_separation(self, current, boats):
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