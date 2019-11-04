#!/usr/bin/env python
import time
import rospy

# how to get current position into program - navdata maybe

from Behaviour_caller import Behave
from ROS_operators.Boat_ID import get_ID
from ROS_operators.Global_data import swarmData
from ROS_operators.Behaviour_talker import Talker

def main():
    BOAT_ID = get_ID()

    data = swarmData()
    behaviour_out = Talker()

    behaviour = Behave(BOAT_ID) #argument for behaviour type

    while not rospy.is_shutdown():
            try:
                data_full = data()
                
                wanted = behaviour(data_full)
                print(wanted)
                behaviour_out(wanted)

                time.sleep(0.7)

            except rospy.ROSInterruptException():
                pass

if __name__=="__main__":
    main()