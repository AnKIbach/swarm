#!/usr/bin/env python
import numpy as np

from Communication import Communication
from PID import PID
from GPS_definition import GPS
from Vector_definition import Vector
from ROS_operators.Navigation_data import navData

class VelocityBehavior:

    def __init__(self, name, maxSpeed, doSpeedRenormalization = False):
        self.name = name
        self.maxSpeed = maxSpeed
        self.doSpeedRenormalization = doSpeedRenormalization
        self.operationArea = None
        self.destination = None

        self.velocity = None
        self.activation = 0.0

    def getName(self):
        return self.name

    def incrementSpeed(self, increment):
        self.maxSpeed += increment

    def updateDestination(self, destination):
        self.destination = destination

    def getMaxSpeed(self):
        return self.maxSpeed

    def getActivation(self):
        return self.activation

    def getVelocity(self):
        return self.velocity

    def updateState(self, body, controlSystem, sensors, state, clock):
        pass

    def evaluate(self, body, controlSystem, sensors, clock):
        self.activation = 0.0
        self.velocity = np.zeros((body.getDim()), dtype = np.double)

    def __call__(self, body, controlSystem, sensors, state, clock):
        self.evaluate(body, controlSystem, sensors, clock)
        vel = self.getVelocity()
        controlSystem.setTargetVelocity(vel)