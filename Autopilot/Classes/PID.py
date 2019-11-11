#!/usr/bin/env python
import sys
import math as m

from  Vector_class import Vector


class PID :
    '''PID class to measure correct speed with minimal delay'''
    def __init__(self): 
        self.Kp=Vector(0.85,1.0) #mindre
        self.Ki=Vector(0.0,0.0) #set for test
        self.Kd=Vector(0.1,0.0) #set for test

        self.Derivator=Vector()
        self.Integrator=Vector()
        self.Integrator_max=Vector(50.0,45.0) #set for test
        self.Integrator_min=Vector(-50.0,-45.0) #set for test
        self.pid_max = Vector(5.0, 45.0)  
        self.wanted_vector=Vector()
        self.error=Vector()
        self.delta_angle = 0.0

    def update(self, current_vector): 
        self.error.magnitude = self.wanted_vector.magnitude - current_vector.magnitude 
        self.error.angle = self._get_delta_angle(current_vector)

        self.P_value =  self.error * self.Kp

        self.D_value =  ( self.error - self.Derivator) * self.Kd 
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error #look at what value we get

        if self.Integrator.magnitude > self.Integrator_max.magnitude or self.Integrator.angle > self.Integrator_max.angle:
            self.Integrator = self.Integrator_max
        elif self.Integrator.magnitude < self.Integrator_min.magnitude or self.Integrator.angle < self.Integrator_min.angle:
            self.Integrator = self.Integrator_min

        
        self.I_value = self.Integrator * self.Ki

        pid = self.P_value + self.I_value + self.D_value

        #linearity fix
        pid.magnitude = current_vector.magnitude + pid.magnitude

        #max speed fix
        if pid.magnitude > self.pid_max.magnitude or pid.magnitude < 0.0: 
            pid.magnitude = self.pid_max.magnitude
        if pid.angle > self.pid_max.angle or pid.angle < -1*self.pid_max.angle:
            pid.angle = self.pid_max.angle

        return pid

    def set_wanted(self, wanted): 
        if isinstance(wanted, Vector) or isinstance(wanted, tuple):
            self.wanted_vector = wanted
            self.Integrator.set(0.0,0.0) #For resetting
        if isinstance(wanted, list) or isinstance(wanted, float):
            self.wanted_vector.set(wanted[0], wanted[1])

    def _set_integrator(self, integrator = Vector(0.0,0.0)):
        self.Integrator = integrator

    def _set_derivator(self, derivator = Vector(0.0,0.0)):
        self.Derivator = derivator

    def _get_delta_angle(self, current_vector):
        deltaXY = m.radians(self.wanted_vector.angle - current_vector.angle )
        self.delta_angle = m.degrees(m.atan2(m.sin(deltaXY), m.cos(deltaXY)))
        return self.delta_angle

