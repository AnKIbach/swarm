from p5 import Vector, stroke, triangle, stroke_weight
import numpy as np
import math as m
import time

class Boid():

    def __init__(self, x, y, width, height):
        self.Ka = 1 #constant alignment
        self.Kc = 1 #constant cohesion
        self.Ks = 1 #constant separation
        self.position = Vector(x, y)
        vec = (np.random.rand(2) - 0.5)*10
        self.movement = Vector(*vec)
        vec = (np.random.rand(2) - 0.5)/2
        self.acc = Vector(*vec)
        self.maxSpeed   = 8.0 # Maximum speed in m/s
        self.maxForce   = 0.3
        self.perception = 100 # Max distance to ...
        self.width = width
        self.height = height

    def show(self): #making boid as a equilateral triangle
        triangle((self.position.x, self.position.y), ((self.position.x +5 ), (self.position.y + 5*m.sqrt(3))), ((self.position.x +10 ), (self.position.y )))

    def edges(self):
        if self.position.x > self.width:
            self.position.x = 0
        elif self.position.x < 0:
            self.position.x = self.width

        if self.position.y > self.height:
            self.position.y = 0
        elif self.position.y < 0:
            self.position.y = self.height

    def align(self, boids):
        alignment = Vector(*np.zeros(2))
        total = 0
        avg_vector = Vector(*np.zeros(2))
        for boid in boids:
            if np.linalg.norm(boid.position - self.position) < self.perception:
                avg_vector += boid.movement
                total += 1
        if total > 0:
            avg_vector /= total
            avg_vector = Vector(*avg_vector)
            avg_vector = (avg_vector / np.linalg.norm(avg_vector)) * self.maxSpeed
            alignment = avg_vector - self.movement

        return alignment

    def cohesion(self, boids):
        cohesion = Vector(*np.zeros(2))
        total = 0
        center_of_mass = Vector(*np.zeros(2))
        for boid in boids:
            if np.linalg.norm(boid.position - self.position) < self.perception:
                center_of_mass += boid.position
                total += 1
        if total > 0:
            center_of_mass /= total
            center_of_mass = Vector(*center_of_mass)
            vec_to_com = center_of_mass - self.position
            if np.linalg.norm(vec_to_com) > 0:
                vec_to_com = (vec_to_com / np.linalg.norm(vec_to_com)) * self.maxSpeed
            cohesion = (vec_to_com - self.movement)
            if np.linalg.norm(cohesion)> self.maxForce:
                cohesion = (cohesion /np.linalg.norm(cohesion)) * self.maxForce
   
        return cohesion

    def separation(self, boids):
        separation = Vector(*np.zeros(2))
        total = 0
        avg_vector = Vector(*np.zeros(2))
        for boid in boids:
            distance = np.linalg.norm(boid.position - self.position)
            if self.position != boid.position and distance < self.perception:
                diff = - (boid.position - self.position)
                diff /= distance
                avg_vector += diff
                total += 1
        if total > 0:
            avg_vector /= total
            avg_vector = Vector(*avg_vector)
            if np.linalg.norm(separation) > 0:
                avg_vector = (avg_vector / np.linalg.norm(separation)) * self.maxSpeed
            separation = avg_vector - self.movement
            if np.linalg.norm(separation) > self.maxForce:
                separation = (separation /np.linalg.norm(separation)) * self.maxForce
        return separation

    def apply_behaviour(self, boids):

        alignment = self.align(boids)
        cohesion = self.cohesion(boids)
        separation = self.separation(boids)

        for boid in boids:
            distance = np.linalg.norm(boid.position - self.position)
            if distance > 10: 
                self.acc += self.Ka * alignment
                self.acc += self.Kc * cohesion
                self.acc += self.Ks * separation   


    def update(self):
        self.position += self.movement
        self.movement += self.acc

        if np.linalg.norm(self.movement) > self.maxSpeed:
            self.movement = self.movement / np.linalg.norm(self.movement) * self.maxSpeed

        self.acc = Vector(*np.zeros(2))