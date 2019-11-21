#!/usr/bin/env python

class Vector:
    def __init__(self, magnitude = 0.0, angle = 0.0):
        self.magnitude = 0.0
        self.angle = 0.0
        if isinstance(magnitude, tuple) or isinstance(magnitude,list): #failsafes - trengs nok ikke
            angle = magnitude[1]
            magnitude = magnitude[0] 
            
        elif isinstance(magnitude, Vector): # samme - failsafes
            angle = magnitude.angle
            magnitude = magnitude.magnitude
            

        self.set(magnitude, angle)

    def set(self, vector_magnitude, vector_angle):
        self.magnitude = vector_magnitude
        self.angle = vector_angle

    def __add__(self, other): # for bruken av + i vectorer
        if isinstance(other, Vector):
            return Vector(self.magnitude + other.magnitude, self.angle + other.angle)
        elif isinstance(other, int) or isinstance(other, float):
            return Vector(self.magnitude + other, self.angle + other)
        else:
            return NotImplemented
        
    def __sub__(self, other): # for bruken av - i vektorer
        if isinstance(other, Vector):
            return Vector(self.magnitude - other.magnitude, self.angle - other.angle)
        elif isinstance(other, int) or isinstance(other, float):
            return Vector(self.magnitude - other, self.angle - other)
        else:
            return NotImplemented

    def __mul__(self, other): # for bruken av - i vektorer
        if isinstance(other, Vector):
            return Vector(self.magnitude * other.magnitude, self.angle * other.angle)
        elif isinstance(other, int) or isinstance(other, float):
            return Vector(self.magnitude * other, self.angle * other)
        else:
            return NotImplemented

    def showVector(self):
        print ("Magnitude: " , round(self.magnitude, 3) , " Angle: " , round(self.angle, 3))