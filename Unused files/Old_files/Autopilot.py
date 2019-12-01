#!/usr/bin/env python
'''
Main program for autopilot node

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


def main():
    nav = navData()
    autopilot = Autopilot()
    time.sleep(0.2)
    autopilot_talker = Talker()
    time.sleep(0.2)
    wait_time, clicks = 0.0, 0
    wanted_GPS = GPS(60.365625, 5.264544) 

    wanted_vector = Vector(0.4, 180.0)

    while nav.is_ready() == False:
    #waits for both systems to connect
        wait_time += 0.1
        time.sleep(0.1)
        if wait_time > 10.0: #exit if timeout is over 10s
            sys.exit(0)

    if nav.is_ready() == True:
        print("Pixhawk is connected and ready at: ", nav.mode)

    print("entering autopilot loop...")

    while not rospy.is_shutdown():
        try:
            # wanted_GPS = new_gps(wanted_GPS)
            
            # fetches newest current speed and position
            current_GPS = nav.get_GPS() 
            current_vector = nav.get_Vector()

            autopilot.set_wanted_vector(wanted_vector)

            change_vector = autopilot(current_vector)
            
            #publishing to ROS
            autopilot_talker(current_vector, 
                            current_GPS,
                            wanted_vector,
                            change_vector)

            clicks += 1

            time.sleep(0.2)

        except rospy.ROSInterruptException():
            sys.exit()
        finally:
            pass

if __name__ == "__main__":
    main()