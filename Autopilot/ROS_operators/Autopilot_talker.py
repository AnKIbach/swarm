#!/usr/bin/env python
import rospy
import math as m

import std_msgs.msg


from GPS_class import GPS
from Vector_class import Vector

from autopilot.msg import test

class talker:
    def __init__(self):
        self.pub_actual = rospy.Publisher('autopilot/actual', test, queue_size=10)



    def __call__(self, wanted, actual, sent):
        wanted.showVector()
        test.speed = wanted.magnitude
        test.angle = wanted.angle
        self.pub_actual.publish(test)

