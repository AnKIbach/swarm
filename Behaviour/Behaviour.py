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
    behaviour = Behave(1, 0)

    time.sleep(2)

    while not rospy.is_shutdown():
            try:
                # do the thing here
                data_full = data()
                data_to_behav = behaviour(data_full)

                P = data_to_behav[0].speed
                print(P)
                
                time.sleep(0.3)

            except rospy.ROSInterruptException():
                pass

if __name__=="__main__":
    main()