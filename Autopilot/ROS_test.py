#!/usr/bin/env python
import time

from PID import PID 
from GPS_class import GPS
from Ranger import autoRange
from Vector_class import Vector



v1 = Vector(10.0,190.0)
v2 = Vector(10.0,190.0)

g1 = GPS(60.366468, 5.258200)
g2 = GPS(60.366468, 5.265216)

v3 = g1.calculate(g2)

v3.showVector()

pid = PID()

pid.set_wanted(v1)

pid_out = pid.update(v2)

pid_out.showVector()

rangespeed = autoRange(0.0,20.0,0.0,90.0)
rangeangle = autoRange(-45.0,45.0,0.0,180.0)
speedout = rangespeed.new(v2.magnitude + pid_out.magnitude) #important - linear v2
angleout = rangeangle.new(pid_out.angle)
print(speedout, angleout)

