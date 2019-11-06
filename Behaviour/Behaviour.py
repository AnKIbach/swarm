#!/usr/bin/env python
import time
import rospy

# how to get current position into program - navdata maybe

from Behaviour_caller import Behave
from ROS_operators.Boat_ID import get_ID
from ROS_operators.Global_data import swarmData
from ROS_operators.Behaviour_sub import Subscriber
from ROS_operators.Behaviour_talker import Talker

def main():
    time_tot = 0.0
    BOAT_ID = get_ID()

    data = swarmData()
    command = Subscriber() #defines a static fence for now

    behaviour_out = Talker()

    # start procedure
    rospy.loginfo("Initiating behaviour, waiting for command..")

    while not command.has_new() and time_tot < 2.0:
        rospy.loginfo("waiting...{}".format(time_tot))
        time.sleep(0.1)
        time_tot += 0.1

    if command.has_new():
        fence = command.fence
        behaviour_type = command.get_taskType

        print(behaviour_type)
        rospy.loginfo_once("Loading behaviour for boat: {}, from command".format(BOAT_ID+1))
        behaviour = Behave(BOAT_ID, fence, behaviour_type) #argument for behaviour type

    else:
        fence = command.standard_fence

        behaviour = Behave(BOAT_ID, fence)

    while not rospy.is_shutdown():
        try:
            data_full = data()
            
            wanted = behaviour(data_full)

            if command.stop():
                rospy.signal_shutdown('stop command recieved')
            else:    
                behaviour_out(wanted)

            time.sleep(0.5)

        except rospy.ROSInterruptException():
            pass

if __name__=="__main__":
    main()