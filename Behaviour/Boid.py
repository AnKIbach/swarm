from p5 import Vector, stroke, triangle, stroke_weight
import numpy as np
import math as m

class Boid():

    def __init__(self, x, y, width, height):
        self.Ka = 1.0
        self.Kc = 1.0
        self.Ks = 1.0
        self.position = Vector(x,y) 
        self.movement = Vector(*np.random.rand(2))
        self.acc      = Vector(*np.random.rand(2))
        self.maxSpeed   = 2.0 # Maximum speed in m/s
        self.maxForce   = 1.0
        self.perception = 100 # Max distance to ...
        self.width = width
        self.height = height

    def show(self): #making boid as a equilateral triangle
        triangle((self.position.x, self.position.y), ((self.position.x +10 ), (self.position.y + 10*m.sqrt(3))), ((self.position.x +20 ), (self.position.y )))

    def edges(self): #to keep the boids on the screen in sim
        if self.position.x > self.width:
            self.position.x = 0
        elif self.position.x < 0:
            self.position.x = self.width
        if self.position.y > self.height:
            self.position.y = 0
        elif self.position.y < 0:
            self.position.y = self.height

    def alignment(self, boats):
        alignment = Vector(0.0,0.0)
        total = 0
        average_vector = Vector(0.0, 0.0)
        for boid in boats:
            if np.linalg.norm(boid.position - self.position) < self.perception and np.linalg.norm(boid.position - self.position) != 0.0:
                average_vector += boid.movement
                total += 1
        if total > 0 and average_vector.magnitude != 0.0:
            alignment = average_vector / total #uncertain of value
            # alignment_tot = m.sqrt(m.pow(alignment.magnitude, 2)+m.pow(alignment.angle, 2))
            alignment_tot = np.linalg.norm(alignment)
            alignment = (alignment / alignment_tot) * self.maxSpeed

        return alignment

    def cohesion(self, boats):
        cohesion = Vector(0.0,0.0)
        total = 0
        center_of_mass = Vector(0.0,0.0)
        for boid in boats:
            if np.linalg.norm(boid.position - self.position) < self.perception and np.linalg.norm(boid.position - self.position) != 0.0:
                center_of_mass += boid.position
                total += 1
        if total > 0 and center_of_mass != 0.0:
            center_of_mass /= total 
            # center_of_mass = Vector(*center_of_mass)
            cohesion = center_of_mass #- self.position
            # cohesion_tot = m.sqrt(m.pow(cohesion.magnitude, 2)+m.pow(cohesion.angle, 2))
            cohesion_tot = np.linalg.norm(cohesion)
            if cohesion_tot > 0: #Makes the vector wanted in proportion with maxSpeed
                cohesion = (cohesion / cohesion_tot) * self.maxSpeed

        return cohesion # vector dowards center of mass

    def separation(self, boats):
        separation = Vector(0.0,0.0)
        total = 0
        average_vector = Vector(0.0,0.0)
        for boid in boats:
            if np.linalg.norm(boid.position - self.position) < self.perception and np.linalg.norm(boid.position - self.position) != 0.0:
                diff = - (boid.position - self.position)
                diff /= np.linalg.norm(boid.position - self.position)
                average_vector += diff
                total += 1
        if total > 0 and average_vector.magnitude != 0.0:
            average_vector /= total
            separation = average_vector #- self.movement
            # separation_tot = m.sqrt(m.pow(separation.magnitude, 2)+m.pow(separation.angle, 2))
            separation_tot=np.linalg.norm(separation)
            if separation_tot > 0:
                separation = (separation / separation_tot) * self.maxForce
        return separation

    def apply_behaviour(self, boats):
        alignment = self.alignment(boats)
        cohesion = self.cohesion(boats)
        separation = self.separation(boats)

        self.acc += alignment * self.Ka + cohesion * self.Kc + separation * self.Ks
        # self.acceleration += alignment + cohesion + separation #force adding
    
    def update(self):
        self.position += self.movement
        self.movement += self.acc

        if np.linalg.norm(self.movement) > self.maxSpeed:
            self.movement = self.movement / np.linalg.norm(self.movement) * self.maxSpeed

        self.acc = Vector(0,0)