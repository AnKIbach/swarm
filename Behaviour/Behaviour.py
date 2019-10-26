#!/usr/bin/env python
import time
import rospy

# how to get current position into program - navdata maybe

from Global_data import swarmData
from Behaviour_caller import Behave


BOAT_ID = 2

def main():
    behaviour = Behave(BOAT_ID)
    data = swarmData()


    while not rospy.is_shutdown():
            try:
                data_full = data()
                # print(data_full)
                print("|||||")

                da = behaviour(data_full)
                print(da)
                time.sleep(0.7)

            except rospy.ROSInterruptException():
                pass

if __name__=="__main__":
    main()