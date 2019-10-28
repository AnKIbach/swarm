#!/usr/bin/env python
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
    behaviour = swarmWanted()
    autopilot = Autopilot()
    autopilot_talker = Talker()
    arduino = Arduino('/dev/ttyACM1', speedLimit = 0.9) #speed limiter for testing
    

    wait_time, clicks = 0.0, 0

    while nav.is_ready() == False and arduino.is_ready() == False:
    #waits for both systems to connect
        wait_time += 0.1
        time.sleep(0.1)
        if wait_time > 10.0: #exit if timeout is over 10s
            sys.exit(0)

    print("Pixhawk is connected and ready at: ", nav.mode)
    print("Arduino is connected and started at: ", arduino.port)
    print("Behaviour is publishing data at: ", behaviour.topic_main)

    print("entering loop...")

    while not rospy.is_shutdown():
        try:
            current_GPS    = nav.get_GPS()
            current_vector = nav.get_Vector()

            if behaviour.is_recieving():
                wanted = behaviour()
                if isinstance(wanted, GPS):
                    wanted_vector = current_GPS.calculate(wanted)
                else:
                    wanted_vector = wanted

            else: #if not recieving from behaviour stop boat
                wanted_vector = Vector(0.0,0.0)

            autopilot.set_wanted_vector(wanted_vector)

            change_vector = autopilot(current_vector)

            print("current vector: ")
            current_vector.showVector()
            print("")
            print("wanted vector: ")
            wanted_vector.showVector()
            print("")
            print("change vector: ")
            change_vector.showVector()
            print("")
            
            arduino(change_vector.magnitude, change_vector.angle) #possible addition of another dampening for angle
            
            #publishing to ROS
            autopilot_talker(current_vector, 
                            current_GPS,
                            wanted_vector,
                            change_vector)

            #publish data after x number of clicks
            if clicks >= 20:
                clicks = 0 
            else:
                clicks += 1

            #comment out for full test
            time.sleep(0.2)

        except rospy.ROSInterruptException():
            arduino() 
            sys.exit()
        finally:
            pass
    arduino()



if __name__ == "__main__":
    main()

