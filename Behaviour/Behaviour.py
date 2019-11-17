#!/usr/bin/env python
import time
import rospy

from Behaviour_caller import Behave, NewCommand
from ROS_operators.Boat_ID import get_ID
from ROS_operators.Global_data import swarmData
from ROS_operators.Behaviour_sub import Subscriber
from ROS_operators.Behaviour_talker import Talker

def main():
    time_tot = 0.0
    BOAT_ID = get_ID()

    data = swarmData()
    command = Subscriber() 
    fence = command.get_static_fence()

    behaviour_out = Talker()

    behaviour = Behave(BOAT_ID, fence, use_behaviour=0) # BOIDS, PSO

    while not rospy.is_shutdown():
        try:
            data_full = data()
            
            wanted = behaviour(data_full)

            if command.stop():
                rospy.signal_shutdown('stop command recieved')
            else:    
                behaviour_out(wanted)

            time.sleep(0.5)

        except NewCommand:
            print("new command")
            if command.stop() == True: 
                rospy.signal_shutdown('stop command recieved')

            else:
                colav = command.get_colavMode()
                if colav == 1: #new behaviour order
                    new_behaviour = command.get_taskType()
                    try:
                        del behaviour
                        behaviour = Behave(BOAT_ID, fence, new_behaviour)
                    except Error as e:
                        ros.loginfo(e)
                        print"could not initiate new behaviour, with error: ", e
                if colav == 2: #new fence
                    pass
                if colav == 3: #new destination
                    pass
                if colav == 4: #new wanted movement
                    pass
            
        except rospy.ROSInterruptException():
            pass

if __name__=="__main__":
    main()


## Working outtake
    # # start procedure
    # rospy.loginfo("Initiating behaviour, waiting for command..")

    # while not command.has_new() and time_tot < 2.0:
    #     rospy.loginfo("waiting...{}".format(time_tot))
    #     time.sleep(0.1)
    #     time_tot += 0.1

    # if command.has_new():
    #     fence = command.get_static_fence #change to new
    #     behaviour_type = command.get_taskType

    #     print(behaviour_type)
    #     rospy.loginfo_once("Loading behaviour for boat: {}, from command".format(BOAT_ID+1))
    #     behaviour = Behave(BOAT_ID, fence, behaviour_type) #argument for behaviour type