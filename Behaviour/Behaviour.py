#!/usr/bin/env python
'''
Written by Andreas Hellesnes, Norwegian Naval Academy 2019

Behaviour.py
This is the main program behaviour,
it gets data from ROS with swarmData and Subscriber
then calculates new movement from behave class
finally publishes to ROS with talker

Questions: anhellesnes@fhs.mil.no
'''

import time
import rospy

from Behaviour_caller import Behave
from ROS_operators.Boat_ID import get_ID
from ROS_operators.Global_data import swarmData
from ROS_operators.Behaviour_sub import Subscriber, NewCommand
from ROS_operators.Behaviour_talker import Talker

def main():
    wait_time = 0.0

    BOAT_ID = get_ID()
    
    #initialise objects for loop
    data = swarmData()
    command = Subscriber() 
    fence = command.get_static_fence()

    behaviour_out = Talker()

    rospy.loginfo("INITIALIZING BEHAVIOUR")
    rospy.loginfo("Waiting for data...")

    while not data.has_recieved():
    #waits to recieve data
        wait_time += 0.1
        time.sleep(0.1)
        if wait_time in (10.0, 20.0, 30.0, 40.0):
            rospy.loginfo("Time waited: {}".format(wait_time))
        elif wait_time > 60.0:
            rospy.loginfo("No data recieved in 60 seconds, behaviour timed out")
            rospy.signal_shutdown('Behaviour timed out')

    rospy.loginfo("Data recieved after time: {}, starting".format(wait_time))

    behaviour = Behave(BOAT_ID, fence, use_behaviour=0) # BOIDS, PSO

    while not rospy.is_shutdown():
        try:
            command()

            time.sleep(0.5)

            #get newest table of data from units in swarm
            data_full = data()
            
            #calculate wanted vector based on current behaviour
            wanted = behaviour(data_full)
 
            #publish wanted vector to autopilot
            behaviour_out(wanted)

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