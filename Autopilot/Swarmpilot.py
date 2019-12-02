#!/usr/bin/env python
'''
Written by Andreas Hellesnes, Norwegian Naval Academy 2019

Swarmpilot.py
This is the main program swarmpilot,
it gets data from ROS with navData and swarmWanted
then calculates new movement with the caller module
finally publishes to ROS with talker

Run this as a node either standalone or as part of system.launch

Questions: anhellesnes@oslo.mil.no
'''

import time
import sys
import rospy

from Classes.GPS_class import GPS
from Classes.Vector_class import Vector
from Classes.Arduino_data import Arduino
from Autopilot_caller import Autopilot

from ROS_operators.Autopilot_sub import swarmWanted
from ROS_operators.Navigation_data import navData
from ROS_operators.Autopilot_talker import Talker
from ROS_operators.Autopilot_datasim import Sim # for simulation


def main():
    #definitin of status dictionary
    status = {'pixhawk': False, 
                'arduino': False, 
                'fix': False, 
                'wifi': False}

    #initiating objects for autopilot
    nav = navData()
    autopilot = Autopilot()
    time.sleep(0.2)
    talker = Talker() # change between for sim or real
    # sim = Sim()
    time.sleep(0.2)
    arduino = Arduino('/dev/Arduino', speedLimit = 0.8) #speed limiter for testing
    time.sleep(0.2)
    behaviour = swarmWanted()
    time.sleep(0.2)

    wait_time, clicks = 0.0, 0

    #waits for both systems to connect
    while not nav.is_ready() and not arduino.is_ready():
        wait_time += 0.1
        time.sleep(0.1)
        if wait_time > 10.0: #exit if timeout is over 10s
            sys.exit(0)

    rospy.loginfo("Pixhawk is connected and ready: {}".format(nav.mode))
    rospy.loginfo("Arduino is connected and started at: {}".format(arduino.port))
    rospy.loginfo("Behaviour is publishing data at: {}".format(behaviour.topic_main))
    
    status = {'pixhawk': True,
                'arduino': True,
                'fix': False,
                'wifi': False}
    
    print("entering loop...")

    while not rospy.is_shutdown():
        try:
            current_GPS    = nav.get_GPS() #gets newest 
            current_vector = nav.get_Vector()

            if behaviour.is_recieving(): # check if behaviour is sending wanted
                wanted = behaviour()
                if isinstance(wanted, GPS):
                    wanted_vector = current_GPS.calculate(wanted)
                else:
                    wanted_vector = wanted
            else: #if not recieving from behaviour stop USV
                print("did not recieve")
                wanted_vector = Vector(0.0, 0.0)

            #sets wanted value for regulator
            autopilot.set_wanted_vector(wanted_vector)

            #calculates vector for new mvoement
            change_vector = autopilot(current_vector)

            #sends vector with new movmement to arduino
            arduino(change_vector.magnitude, change_vector.angle) #possible addition

            #publishes current data to ROS
            talker(current_vector, current_GPS, wanted_vector, change_vector) #change between for sim or real
            # sim()

            #publishes status of USV every 20 clicks 
            if clicks >= 20:
                talker.publish_status(status) 
                clicks = 0
            else:
                clicks += 1
            #comment out for full test
            time.sleep(0.5)

        except rospy.ROSInterruptException():
            arduino() 
            sys.exit()
        finally:
            pass
    arduino() # to reset boat when exiting node

if __name__ == "__main__":
    main()