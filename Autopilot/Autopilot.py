#!/usr/bin/env python
import time
import sys

import rospy
from GPS_class import GPS
from Vector_class import Vector
from Arduino_data import Arduino
from Autopilot_call import Autopilot
from ROS_operators.Navigation_data import navData
from ROS_operators.Autopilot_talker import talker


def main():
    nav = navData()
    autopilot = Autopilot() 
    #uncomment for test
    #arduino = Arduino(speedLimit = 0.5) #speed limiter for testing
    #arduino.connect()
    #arduino.neutral_start()
    
    autopilot_talker = talker()

    wanted_GPS = GPS(60.394087, 5.266185)

    wait_time = 0.0

    while nav.get_connection_state() == False: #or arduino.is_ready() == False:
    #waits for both systems to connect
        wait_time += 0.1
        time.sleep(0.1)
        if wait_time > 10.0: #exit if timeout is over 10s
            sys.exit(0)

    while True:
        try:
            current_GPS = nav.get_GPS()
            current_vector = nav.get_Vector()

            wanted_vector = wanted_GPS.calculate(current_GPS)

            autopilot.set_wanted_vector(wanted_vector)

            change = autopilot(current_vector)
            print("change")
            change.showVector()

            #arduino.update(change.magnitude, -change.angle)
            autopilot_talker(wanted_vector,current_vector, change)

            time.sleep(0.1)

        
        except rospy.ROSInterruptException():
            sys.exit()
        finally:
            print(1)



if __name__ == "__main__":
    main()

