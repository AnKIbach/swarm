#!/usr/bin/env python
import time
import rospy

# how to get current position into program - navdata maybe

from Behaviour_caller import Behave
from ROS_operators.Global_data import swarmData

BOAT_ID = 2

def main():
    behaviour = Behave(BOAT_ID) #argument for behaviour type 
    data = swarmData()

    while not rospy.is_shutdown():
            try:
                data_full = data()
                
                da = behaviour(data_full)
                print(da)
                time.sleep(0.7)

            except rospy.ROSInterruptException():
                pass

if __name__=="__main__":
    main()