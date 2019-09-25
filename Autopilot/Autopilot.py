#!/usr/bin/env python
import time
import sys
import rospy

from GPS_class import GPS
from PID_plotter import Plotter
from Vector_class import Vector
from Arduino_data import Arduino
from Autopilot_call import Autopilot
from ROS_operators.Navigation_data import navData
from ROS_operators.Navigation_data import newGPS #newGPS for testing
from ROS_operators.Autopilot_talker import Talker


def main():
    nav = navData()
    plt = Plotter()
    new_gps = newGPS() 
    autopilot = Autopilot()
    
    #uncomment for test
    #arduino = Arduino(speedLimit = 0.5) #speed limiter for testing
    #arduino.connect()
    #arduino.neutral_start()
    
    #fix for test
    autopilot_talker = Talker()

    sOut, sAct, aOut, aAct = [], [], [], []
    wait_time, clicks = 0.0, 0

    wanted_GPS = GPS(60.394087, 5.266185)
    wanted_GPS.show()
    time.sleep(2.0)

    while nav.get_connection_state() == False: #or arduino.is_ready() == False:
    #waits for both systems to connect
        wait_time += 0.1
        time.sleep(0.1)
        if wait_time > 10.0: #exit if timeout is over 10s
            sys.exit(0)

    while True:
        try:
            wanted_GPS = new_gps.update(wanted_GPS)

            print("wanted GPS")
            wanted_GPS.show()

            current_GPS = nav.get_GPS()
            current_vector = nav.get_Vector()
            
            print("GPS current")
            current_GPS.show()

            wanted_vector = current_GPS.calculate(wanted_GPS)

            autopilot.set_wanted_vector(wanted_vector)

            change = autopilot(current_vector)

            print("change vector")
            change.showVector()
            #arduino.update(change.magnitude, -change.angle)
            autopilot_talker(current_vector, current_GPS) 

            #for visualisation of values:
            sOut.append(wanted_vector.magnitude)
            aOut.append(wanted_vector.angle)
            sAct.append(current_vector.magnitude)
            aAct.append(current_vector.angle)

            if clicks <= 100:
                plt.present(sOut, sAct, aOut, aAct)
                clicks = 0 
            else:
                clicks += 1

            time.sleep(1.0)

        except rospy.ROSInterruptException():
            sys.exit()
        finally:
            pass



if __name__ == "__main__":
    main()

