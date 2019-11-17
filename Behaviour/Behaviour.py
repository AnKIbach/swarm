#!/usr/bin/env python
import time
import rospy

from Behaviour_caller import Behave
from ROS_operators.Boat_ID import get_ID
from ROS_operators.Global_data import swarmData
from ROS_operators.Behaviour_sub import Subscriber, NewCommand
from ROS_operators.Behaviour_talker import Talker

def main():
    BOAT_ID = get_ID()

    data = swarmData()
    command = Subscriber() 
    fence = command.get_static_fence()

    behaviour_out = Talker()

    rospy.loginfo("INITIALIZING BEHAVIOUR")
    time.sleep(4)

    behaviour = Behave(BOAT_ID, fence, use_behaviour=0) # BOIDS, PSO

    while not rospy.is_shutdown():
        try:
            command()

            data_full = data()
            
            wanted = behaviour(data_full)
 
            behaviour_out(wanted)

            time.sleep(0.5)

        except NewCommand:
            if command.stop() == True: 
                rospy.signal_shutdown('stop command recieved')

            else:
                colav = command.get_colavMode()
                if colav == 1: #new behaviour order
                    new_behaviour = command.get_taskType()
                    try:
                        rospy.loginfo("Initiating new behaviour...")
                        del behaviour
                        behaviour = Behave(BOAT_ID, fence, new_behaviour)
                    except AttributeError as e:
                        rospy.loginfo("could not initiate new behaviour, with error: {} ", format(e))
                
                if colav == 2: #new fence
                    print("trying to set new fence")
                    new_fence = command.get_fence()

                    behaviour.change_fence(new_fence)

                if colav == 3: #new destination
                    print("trying to set new dest")
                    destination = command.get_wantedPos()
                    
                    behaviour.set_destination(destination)

                if colav == 4: #new wanted movement
                    pass #not in use for any beahviour pr now
            
        except rospy.ROSInterruptException():
            pass

if __name__=="__main__":
    main()