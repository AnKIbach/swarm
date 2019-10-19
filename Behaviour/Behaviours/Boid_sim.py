from p5 import Vector, stroke, triangle, stroke_weight
import numpy as np
import math

class Boid():

    def __init__(self, x, y, width, height):
        self.position = Vector(x, y) 
        self.velocity = Vector(*np.random.rand(2)) 
        self.acceleration = Vector(*np.random.rand(2)) 
        self.maxForce = 0.3 # to control the magnitude of cohesion and separation
        self.maxSpeed = 15 
        self.perception = 200 #max distance to scan for other boids
        self.width = width
        self.height = height

    def show(self): #making boid as a equilateral triangle
        triangle((self.position.x, self.position.y), ((self.position.x +10 ), (self.position.y + 10*math.sqrt(3))), ((self.position.x +20 ), (self.position.y )))

    def edges(self): #to keep the boids on the screen in sim
        if self.position.x > self.width:
            self.position.x = 0
        elif self.position.x < 0:
            self.position.x = self.width
        if self.position.y > self.height:
            self.position.y = 0
        elif self.position.y < 0:
            self.position.y = self.height

    def alignment(self, boids):
        steering = Vector(0,0)
        total = 0
        average_vector = Vector(0,0)
        for boid in boids:
            if np.linalg.norm(boid.position - self.position) < self.perception:
                average_vector += boid.velocity
                total += 1
        if total > 0:
            average_vector = Vector(*average_vector/total)
            average_vector = (average_vector / np.linalg.norm(average_vector)) * self.maxSpeed
            steering = average_vector - self.velocity
        return steering 

    def cohesion(self, boids):
        steering = Vector(0,0) 
        total = 0
        center_of_mass = Vector(0,0)
        for boid in boids:
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

    def separation(self, boids):
        steering = Vector(0,0)
        total = 0
        average_vector = Vector(0,0)
        for boid in boids:
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

    def apply_behaviour(self, boids):
        alignment = self.alignment(boids)
        cohesion = self.cohesion(boids)
        separation = self.separation(boids)

        self.acceleration += alignment + cohesion + separation #force adding
    
    def update(self):
        self.position += self.velocity
        self.velocity += self.acceleration
    
        if np.linalg.norm(self.velocity) > self.maxSpeed:
            self.velocity = self.velocity / np.linalg.norm(self.velocity) * self.maxSpeed
        self.acceleration = Vector(0,0)