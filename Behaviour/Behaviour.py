#!/usr/bin/env python
import time
import rospy

# how to get current position into program - navdata maybe

from Global_data import swarmData
from Behaviour_caller import Behave


BOAT_ID = 2

def main():
    data = swarmData()
    behaviour = Behave(BOAT_ID, 0)

    while not rospy.is_shutdown():
            try:
                # do the thing here
                data_full = data()
                data_to_behav = behaviour(data_full)

                P = data_to_behav[0]['distance']
                print(P)

                time.sleep(0.5)

            except rospy.ROSInterruptException():
                pass

if __name__=="__main__":
    main()