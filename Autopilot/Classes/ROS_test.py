#!/usr/bin/env python
import time

from Classes.PID import PID 
from Classes.GPS_class import GPS
from Classes.Ranger import autoRange
from Classes.Vector_class import Vector



v1 = Vector(10.0,190.0)
v2 = Vector(10.0,190.0)

g1 = GPS(60.366468, 5.258200)
g2 = GPS(60.366455, 5.258232)

v3 = g1.calculate(g2)

v3.showVector()

pid = PID()

pid.set_wanted(v1)

pid_out = pid.update(v2)

pid_out.showVector()

rangespeed = autoRange(0.0,20.0,0.0,90.0)
rangeangle = autoRange(-45.0,45.0,0.0,180.0)

new = rangeangle.new(0.05)

print(new)

