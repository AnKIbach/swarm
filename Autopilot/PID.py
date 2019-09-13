#!/usr/bin/env python
from Vector_definition import Vector
import math as m

class PID :
    def __init__(self): 
        self.Kp=Vector(0.1,0.1)
        self.Ki=Vector(0.0,0.0)
        self.Kd=Vector(0.0,0.0)

        self.Derivator=Vector()
        self.Integrator=Vector()
        self.Integrator_max=Vector(500,360)
        self.Integrator_min=Vector(-500,0)
        self.pid_max = Vector(20.0, 360) 
        self.wanted_vector=Vector()
        self.error=Vector()
        self.delta_angle = 0.0

    def update(self, current_vector): 
        self.error.magnitude = self.wanted_vector.magnitude - current_vector.magnitude 
        self.error.angle = self._get_delta_angle(current_vector)

        self.P_value =  self.error * self.Kp

        self.D_value =  ( self.error - self.Derivator) * self.Kd 
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator.magnitude > self.Integrator_max.magnitude or self.Integrator.angle > self.Integrator_max.angle:
            self.Integrator = self.Integrator_max
        elif self.Integrator.magnitude < self.Integrator_max.magnitude or self.Integrator.angle < self.Integrator_max.angle:
            self.Integrator = self.Integrator_min

        
        self.I_value = self.Integrator * self.Ki

        pid = self.P_value + self.I_value + self.D_value

        if pid.magnitude > self.pid_max.magnitude or pid.magnitude < -1*self.pid_max.magnitude: 
            pid.magnitude = self.pid_max.magnitude
        if pid.angle > self.pid_max.angle or pid.angle < -1*self.pid_max.angle:
            pid.angle = self.pid_max.angle

        return pid

    def set_wanted(self, wanted_vector): 
        self.wanted_vector = wanted_vector


    def _set_integrator(self, integrator = Vector(0.0,0.0)):
        self.Integrator = integrator

    def _set_derivator(self, derivator = Vector(0.0,0.0)):
        self.Derivator = derivator

    def _get_delta_angle(self, current_vector):
        deltaXY = m.radians(self.wanted_vector.angle - current_vector.angle )
        self.delta_angle = m.degrees(m.atan2(m.sin(deltaXY), m.cos(deltaXY)))
        return self.delta_angle
