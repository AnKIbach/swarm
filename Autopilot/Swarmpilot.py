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
#for sim
from ROS_operators.Autopilot_datasim import Sim


def main():
    status = {'pixhawk': False, 
                'arduino': False, 
                'fix': False, 
                'wifi': False}

    nav = navData()
    autopilot = Autopilot()
    time.sleep(0.2)
    # talker = Talker()
    sim = Sim()
    time.sleep(0.2)
    arduino = Arduino('/dev/Arduino', speedLimit = 0.8) #speed limiter for testing
    time.sleep(0.2)
    behaviour = swarmWanted()
    time.sleep(0.2)

    wait_time, clicks = 0.0, 0

    while not nav.is_ready() and not arduino.is_ready():
    #waits for both systems to connect
        wait_time += 0.1
        time.sleep(0.1)
        if wait_time > 10.0: #exit if timeout is over 10s
            sys.exit(0)

    print("Pixhawk is connected and ready and: ", nav.mode)
    print("Arduino is connected and started at: ", arduino.port)
    print("Behaviour is publishing data at: ", behaviour.topic_main)
    
    status = {'pixhawk': True,
                'arduino': True,
                'fix': False,
                'wifi': False}

    # talker.publish_status(status) Not working
    
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
                print("did not recieve")
                wanted_vector = Vector(0.0, 0.0)

            autopilot.set_wanted_vector(wanted_vector)

            change_vector = autopilot(current_vector)

            print("autopilot")
            print("current vector: ")
            current_vector.showVector()
            print("wanted vector: ")
            wanted_vector.showVector()
            print("change vector: ")
            change_vector.showVector()
            print("")

            arduino(change_vector.magnitude, change_vector.angle) #possible addition

            # talker(current_vector, current_GPS, wanted_vector, change_vector)
            sim()
            
            if clicks >= 20:
                # talker.publish_status(status) not working
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
    arduino()

if __name__ == "__main__":
    main()