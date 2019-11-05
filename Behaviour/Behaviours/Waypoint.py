#!/usr/bin/env python
import numpy as np

from Classes.GPS_class import GPS

from velBehav import VelocityBehavior

class waypointBehaviour(VelocityBehavior):

    def __init__(self, waypoint, maxSpeed):
        VelocityBehavior.__init__(self, "Waypoint", maxSpeed, doSpeedRenormalization = True) #henter inn klassen fra BehaviourVelocity
        self.destination = waypoint

    def evaluate(self, body, controlSystem, sensors, clock):

        self.activation = 1.0

        vector2Dest = self.destination - body.getPos()

        dist = np.sqrt( np.sum( vector2Dest[:]**2 ) )

        self.velocity = vector2Dest / (dist + 1e-12) * self.maxSpeed 