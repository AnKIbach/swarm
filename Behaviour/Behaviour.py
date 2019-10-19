#!/usr/bin/env python
import time
import rospy

# how to get current position into program - navdata maybe

from Global_data import swarmData
from Behaviour_caller import Behave

from Behaviours.Classes.GPS_class import GPS
from Behaviours.Classes.Vector_class import Vector

def main():
    data = swarmData()
    behaviour = Behave("Boids")

    time.sleep(2)

    while not rospy.is_shutdown():
            try:
                # do the thing here
                listest = data()
                p = listest[1].movement.bearing

                time.sleep(0.1)

            except rospy.ROSInterruptException():
                pass

if __name__=="__main__":
    main()