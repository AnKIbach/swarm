#!/usr/bin/env python
import time
import rospy

from Behaviours.Boid import Boids
from Classes.GPS_class import GPS
from Classes.Vector_class import Vector

def main():
    data = swarmData()

    time.sleep(2)

    while not rospy.is_shutdown():
            try:
                listest = data()
                p = listest[1].movement.bearing
                print(p)
                time.sleep(0.1)

            except rospy.ROSInterruptException():
                pass

if __name__=="__main__":
    main()